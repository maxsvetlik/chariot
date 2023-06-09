# type:ignore
# We ignore typing on this file because of the lack of support for typing against the can messaging system and specifically
# the interaction with the cansimple.dbc.

"""
Copyright (c) 2023 Maxwell Svetlik

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

"""
This is a non-comprehensive interface for interaction with the CANSimple protocol that is used by the Odrive Pro and
Odrive S1. The protocol details can be found at https://docs.odriverobotics.com/v/0.5.4/can-protocol.html.
CANSimple is compliant with the Linux kernel's socketcan, which is used in this library. This restricts the use
of this to machines running Linux.

Note that there are commands that are supported on the ODrive that is not yet implemented here, and there are more
commands supported on the ODrive USB interface than are supported on the CAN interface.

N.B. this interface uses `odrive.ctrl_enums` which is subject to change after firmware updates. Therefore, this
library may break after a firmware upgrade.
"""

import dataclasses
import enum
import os
from typing import Any, Protocol

import can
import cantools
import func_timeout
import odrive.enums as ctrl_enums
import structlog

LOG = structlog.get_logger()
CAN_BUS = (  # socketcan is explicitly supported by CANSimple. Unfortunately this limits use to Linux.
    "socketcan"
)


class ArbitrationCommand(enum.Enum):
    # Note that CANOpen addresses are reserved to prevent bus collisions
    NMT_MESSAGE = 0x000
    ODRIVE_HEARTBEAT_MESSAGE = (
        0x001  # controller_flags, encoder_flags, motor_flags, axis_state, axis_error
    )
    ESTOP_MESSAGE = 0x002  # Not yet implemented
    GET_MOTOR_ERROR = 0x003
    GET_ENCODER_ERROR = 0x004
    GET_SENSORLESS_ERROR = 0x005
    SET_AXIS_NODE_ID = 0x006
    SET_AXIS_REQUESTED_STATE = 0x007
    SET_AXIS_STARTUP_CONFIG = 0x008  # Not yet implemented
    GET_ENCODER_ESTIMATE = 0x009  # Encoder Pos Estimate Encoder Vel Estimate
    GET_ENCODER_COUNT = 0x00A  # Encoder Shadow Count Encoder Count in CPR
    SET_CONTROLLER_MODE = 0x00B  # Control Mode Input Mode
    SET_INPUT_POS = 0x00C
    SET_INPUT_VEL = 0x00D
    SET_INPUT_TORQUE = 0x00E
    SET_LIMITS = 0x00F  # Velocity Limit Current Limit
    START_ANTICLOGGING = 0x010
    SET_TRAJ_VEL_LIMIT = 0x011
    SET_TRAJ_ACCEL_LIMITS = 0x012  # Traj Accel Limit Traj Decel Limit
    SET_TRAJ_INERTIA = 0x013
    GET_IQ = 0x014  # it ain't high!
    GET_SENSORLESS_ESTIMATES = 0x015  # Sensorless Pos Estimate Sensorless Vel Estimate
    REBOOT = 0x016
    GET_VBUS_VOLTAGE = 0x017
    CLEAR_ERRORS = 0x018
    SET_LINEAR_COUNT = 0x019
    SET_POSITION_GAIN = 0x01A
    SET_VEL_GAIN = 0x01B  # Vel Gain Vel Integrator Gain
    CANOPEN_HEARTBEAT_MESSAGE = 0x700


@dataclasses.dataclass
class EncoderEstimateMsg:
    vel_estimate: float
    pos_estiamte: float


@dataclasses.dataclass
class HeartBeatMsg:
    controller_flags: str  # See ctrl_enums.ControllerError
    encoder_flags: str  # See ctrl_enums.EncoderError
    motor_flags: str  # See ctrl_enums.MotorError
    axis_state: str  # See ctrl_enums.AxisState
    axis_error: str  # See ctrl_enums.AxisError


@dataclasses.dataclass
class CanProtocolSettings(Protocol):
    device_id: str
    bitrate: str


class MessageSendFailure(Exception):
    pass


class CANDeviceNotFound(Exception):
    pass


class CanSimpleInterface:
    """An interface for communicating with devices that support the CANSimpler CAN protocol. It is expected that
    the CAN device is up and available.

    :param protocol_settings: the specification for connecting to the CAN bus.
    """

    def __init__(
        self, protocol_settings: CanProtocolSettings, receive_timeout=1.0
    ) -> None:
        self._protocol_settings = protocol_settings
        self._db = cantools.database.load_file(
            os.path.dirname(__file__) + "/can/odrive-cansimple.dbc"
        )
        self._receive_timeout = receive_timeout

        try:
            self._bus = can.interface.Bus(
                channel=self._protocol_settings.device_id, bustype="socketcan"
            )
        except OSError:
            LOG.error("Could not find interface")
            raise CANDeviceNotFound

    def send(self, axis_id: int, arbitration_id: ArbitrationCommand, data: Any) -> None:
        try:
            msg = can.Message(
                arbitration_id=arbitration_id.value | axis_id << 5,
                is_extended_id=False,
                data=data,
            )
            self._bus.send(msg)
        except can.CanError:
            LOG.warn("Message NOT sent!")
            raise MessageSendFailure

    """AXIS STATE"""

    def _set_axis_state(self, axis_id: int, axis_state: ctrl_enums.AxisState) -> None:
        data = self._db.encode_message(
            "Set_Axis_State", {"Axis_Requested_State": axis_state}
        )
        self.send(axis_id, ArbitrationCommand.SET_AXIS_REQUESTED_STATE, data)

    def set_axis_closed_loop(self, axis_id: int):
        # An axis in a CLOSED_LOOP state will carry out movement commands that are sent. Some parameters may only
        # be set in when an axis is in IDLE mode.
        self._set_axis_state(axis_id, ctrl_enums.AxisState.CLOSED_LOOP_CONTROL)

    def set_axis_idle(self, axis_id: int):
        # An axis in an idle state will not respond to movement commands, even though they may still be sent.
        self._set_axis_state(axis_id, ctrl_enums.AxisState.IDLE)

    """CONTROL MODES"""

    def _set_control_mode(
        self,
        axis_id: int,
        ctrl_mode: ctrl_enums.ControlMode,
        input_mode: ctrl_enums.InputMode = ctrl_enums.InputMode.INACTIVE,
    ):
        """Sets the control and input mode, if the ctrl mode should support it. Learn more about Input Modes:
        https://docs.odriverobotics.com/v/latest/control-modes.html

        :param axis_id: the can ID of the controller in which to send the command.
        :param ctrl_mode: the requested control mode for the controller.
        :param input_mode: a sub mode of the ctrl mode. Available input modes are different for each ctrl mode.
        """
        data = self._db.encode_message(
            "Set_Controller_Mode",
            {"Input_Mode": input_mode.value, "Control_Mode": ctrl_mode.value},
        )
        self.send(axis_id, ArbitrationCommand.SET_AXIS_REQUESTED_STATE, data)

    def set_velocity_control_mode(self, axis_id: int):
        """Sets the velocity control mode with Velocity Ramp as the input mode."""
        self._set_control_mode(
            axis_id,
            ctrl_enums.ControlMode.VELOCITY_CONTROL,
            ctrl_enums.InputMode.VEL_RAMP,
        )

    def set_position_control_mode(
        self,
        axis_id: int,
        input_mode: ctrl_enums.InputMode = ctrl_enums.InputMode.INACTIVE,
    ):
        self._set_control_mode(
            axis_id, ctrl_enums.ControlMode.POSITION_CONTROL, input_mode
        )

    def set_torque_control_mode(
        self,
        axis_id: int,
        input_mode: ctrl_enums.InputMode = ctrl_enums.InputMode.INACTIVE,
    ):
        self._set_control_mode(
            axis_id, ctrl_enums.ControlMode.TORQUE_CONTROL, input_mode
        )

    def set_voltage_control_mode(
        self,
        axis_id: int,
        input_mode: ctrl_enums.InputMode = ctrl_enums.InputMode.INACTIVE,
    ):
        self._set_control_mode(
            axis_id, ctrl_enums.ControlMode.VOLTAGE_CONTROL, input_mode
        )

    """MOVEMENT TARGETS"""

    def set_target_velocity(
        self, axis_id: int, target_vel: float, torque_ff: float = 0.0
    ) -> None:
        """It is required that the ODrive is in velocity control mode for this command to be meaningful."""
        data = self._db.encode_message(
            "Set_Input_Vel", {"Input_Vel": target_vel, "Input_Torque_FF": torque_ff}
        )
        self.send(axis_id, ArbitrationCommand.SET_INPUT_VEL, data)

    def set_target_position(
        self,
        axis_id: int,
        set_point: float,
        vel_ff: float = 0.0,
        torque_ff: float = 0.0,
    ) -> None:
        """Set the target position for the controller when it is in position control mode. If the controller
        is in a different control mode, this command will have no effect.

        :param axis_id: the can ID of the controller in which to send the command.
        :param set_point: the commanded position of the controller.
        :param vel_ff: the velocity feed forward controller parameter.
        :param torque_ff: the torque feed forward controller parameter.
        """

        data = self._db.encode_message(
            "Set_Input_Pos",
            {"Input_Pos": set_point, "Vel_FF": vel_ff, "Torque_FF": torque_ff},
        )
        self.send(axis_id, ArbitrationCommand.SET_INPUT_POS, data)

    def clear_motor_error(self, axis_id: int) -> None:
        """Clear all set errors for a given axis ID.

        :param axis_id: the can ID of the controller in which to send the command.
        """

        data = self._db.encode_message("Clear_Errors", {})
        self.send(axis_id, ArbitrationCommand.CLEAR_ERRORS, data)

    """RECEIVE STATE"""

    @func_timeout.func_set_timeout(5.0)
    def receive(self, axis_id: int, arbitration_cmd: Any) -> can.Message:
        """Receive a specific message from the bus.
        :param axis_id: the can ID of the controller in which to send the command.
        :param arbitrarion_cmd: the message command sent on the bus for data retrieval.
        """
        while True:
            msg = self._bus.recv(self._receive_timeout)
            if msg.arbitration_id == ((axis_id << 5) | arbitration_cmd):
                return msg

    def get_encoder_estimate(self, axis_id: int) -> EncoderEstimateMsg:
        # As of firmware verion 0.5.4 this message type is sent on the bus at 100Hz
        rx = self.receive(
            axis_id, self._db.get_message_by_name("Get_Encoder_Estimates").frame_id
        )
        msg = self._db.decode_message("Get_Encoder_Estimates", rx.data)  # type:ignore
        return EncoderEstimateMsg(msg["Vel_Estimate"], msg["Pos_Estimate"])

    def get_heartbeat(self, axis_id: int) -> HeartBeatMsg:
        """Get the heartbeat message.

        :raises: FunctionTimedOut
        """
        rx = self.receive(axis_id, self._db.get_message_by_name("Heartbeat").frame_id)
        msg = self._db.decode_message("Heartbeat", rx.data)  # type:ignore
        return HeartBeatMsg(
            hex(msg["Controller_Flags"]),
            hex(msg["Encoder_Flags"]),
            hex(msg["Motor_Flags"]),
            hex(msg["Axis_State"]),
            hex(msg["Axis_Error"]),
        )

    def get_bus_voltage(self, axis_id: int) -> float:
        """Get the bus voltage.

        :raises: FunctionTimedOut
        """
        self.send(axis_id, ArbitrationCommand.GET_VBUS_VOLTAGE, {})
        rx = self.receive(
            axis_id, self._db.get_message_by_name("Get_Vbus_Voltage").frame_id
        )
        msg = self._db.decode_message("Get_Vbus_Voltage", rx.data)  # type:ignore
        return msg["Vbus_Voltage"]

    def get_motor_error(self, axis_id: int) -> str:
        """Get axis errors from the motor, if any.

        :raises: FunctionTimedOut
        """
        self.send(axis_id, ArbitrationCommand.GET_MOTOR_ERROR, {})
        rx = self.receive(
            axis_id, self._db.get_message_by_name("Get_Motor_Error").frame_id
        )
        msg = self._db.decode_message("Get_Motor_Error", rx.data)  # type:ignore
        return hex(msg["Motor_Error"])

    def get_encoder_error(self, axis_id: int) -> str:
        """Get encoder error. N.B. cantools doesn't support use of the RTR feature of CAN, which is
        required for this data's CAN message, so this message is scraped from the Heartbeat.
                :raises: FunctionTimedOut
        """

        heartbeat = self.get_heartbeat(axis_id)
        return heartbeat.encoder_flags
