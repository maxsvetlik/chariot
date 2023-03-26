import dataclasses
import enum
import os
from typing import Any, Protocol

import can
import cantools
import func_timeout
import odrive.enums as ctrl_enums

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
    controller_flags: ctrl_enums.ControllerError
    encoder_flags: ctrl_enums.EncoderError
    motor_flags: ctrl_enums.MotorError
    axis_state: ctrl_enums.AxisState
    axis_error: ctrl_enums.AxisError


@dataclasses.dataclass
class MotorState:
    estimates: EncoderEstimateMsg
    heartbeat: HeartBeatMsg
    voltage: float


@dataclasses.dataclass
class CanProtocolSettings(Protocol):
    device_id: str
    bitrate: int


class MessageSendFailure(Exception):
    pass


class CANDeviceNotFound(Exception):
    pass


@dataclasses.dataclass
class MotorStateMsg:
    pass


class CanSimpleInterface:
    """An interface for communicating with devices that support the CANSimpler CAN protocol. It is expected that
    the CAN device is up and available.

    :param protocol_settings: the specification for connecting to the CAN bus.
    """

    def __init__(self, protocol_settings: CanProtocolSettings) -> None:
        self._protocol_settings = protocol_settings
        self._db = cantools.database.load_file(
            os.path.dirname(__file__) + "/can/odrive-cansimple.dbc"
        )

        try:
            self._bus = can.interface.Bus(
                channel=self._protocol_settings.device_id, bustype="socketcan"
            )
        except OSError:
            print("Could not find interface")
            raise CANDeviceNotFound

    def send(
        self, axis_id: bytes, arbitration_id: ArbitrationCommand, data: Any
    ) -> None:
        try:
            msg = can.Message(
                arbitration_id=arbitration_id.value | axis_id << 5,
                is_extended_id=False,
                data=data,
            )
            self._bus.send(msg)
        except can.CanError:
            print("Message NOT sent!")
            raise MessageSendFailure

    """AXIS STATE"""

    def _set_axis_state(self, axis_id: bytes, axis_state: ctrl_enums.AxisState) -> None:
        data = self._db.encode_message(
            "Set_Axis_State", {"Axis_Requested_State": axis_state}
        )
        self.send(axis_id, ArbitrationCommand.SET_AXIS_REQUESTED_STATE, data)

    def set_axis_closed_loop(self, axis_id: bytes):
        # An axis in a CLOSED_LOOP state will carry out movement commands that are sent. Some parameters may only
        # be set in when an axis is in IDLE mode.
        self._set_axis_state(axis_id, ctrl_enums.AxisState.CLOSED_LOOP_CONTROL)

    def set_axis_idle(self, axis_id: bytes):
        # An axis in an idle state will not respond to movement commands, even though they may still be sent.
        self._set_axis_state(axis_id, ctrl_enums.AxisState.IDLE)

    """CONTROL MODES"""

    def _set_control_mode(
        self,
        axis_id: bytes,
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

    def set_velocity_control_mode(self, axis_id: bytes):
        """Sets the velocity control mode with Velocity Ramp as the input mode."""
        self._set_control_mode(
            axis_id,
            ctrl_enums.ControlMode.VELOCITY_CONTROL,
            ctrl_enums.InputMode.VEL_RAMP,
        )

    def set_position_control_mode(
        self,
        axis_id: bytes,
        input_mode: ctrl_enums.InputMode = ctrl_enums.InputMode.INACTIVE,
    ):
        self._set_control_mode(
            axis_id, ctrl_enums.ControlMode.POSITION_CONTROL, input_mode
        )

    def set_torque_control_mode(
        self,
        axis_id: bytes,
        input_mode: ctrl_enums.InputMode = ctrl_enums.InputMode.INACTIVE,
    ):
        self._set_control_mode(
            axis_id, ctrl_enums.ControlMode.TORQUE_CONTROL, input_mode
        )

    def set_voltage_control_mode(
        self,
        axis_id: bytes,
        input_mode: ctrl_enums.InputMode = ctrl_enums.InputMode.INACTIVE,
    ):
        self._set_control_mode(
            axis_id, ctrl_enums.ControlMode.VOLTAGE_CONTROL, input_mode
        )

    """MOVEMENT TARGETS"""

    def set_target_velocity(
        self, axis_id: bytes, target_vel: float, torque_ff: float = 0.0
    ) -> None:
        """It is required that the ODrive is in velocity control mode for this command to be meaningful."""
        data = self._db.encode_message(
            "Set_Input_Vel", {"Input_Vel": target_vel, "Input_Torque_FF": torque_ff}
        )
        self.send(axis_id, ArbitrationCommand.SET_INPUT_VEL, data)

    def set_target_position(
        self,
        axis_id: bytes,
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

    def clear_motor_error(self, axis_id: bytes) -> None:
        """Clear all set errors for a given axis ID.

        :param axis_id: the can ID of the controller in which to send the command.
        """

        data = self._db.encode_message("CLEAR_ERRORS")
        self.send(axis_id, ArbitrationCommand.CLEAR_ERRORS, data)

    """RECEIVE STATE"""

    @func_timeout.func_set_timeout(5.0)
    def receive(self, axis_id: bytes, arbitration_cmd: Any) -> can.Message:
        """Receive a specific message from the bus.
        :param axis_id: the can ID of the controller in which to send the command.
        :param arbitrarion_cmd: the message command sent on the bus for data retrieval."""
        while True:
            msg = can.bus.recv(self._receive_timeout)
            if msg.arbitration_id == ((axis_id << 5) | arbitration_cmd):
                return msg

    def get_encoder_estimate(self, axis_id: bytes) -> EncoderEstimateMsg:
        # As of firmware verion 0.5.4 this message type is sent on the bus at 100Hz
        rx = self.receive(
            axis_id, self._db.get_message_by_name("Get_Encoder_Estimates").frame_id
        )
        msg = self._db.decode_message("Heartbeat", rx.data)  # type:ignore
        return EncoderEstimateMsg(msg["Vel_Estimate"], msg["Pos_Estimate"])

    def get_heartbeat(self, axis_id) -> HeartBeatMsg:
        """Get the heartbeat message.

        :raises: FunctionTimedOut
        """
        rx = self.receive(axis_id, self._db.get_message_by_name("Heartbeat").frame_id)
        msg = self._db.decode_message("Heartbeat", rx.data)  # type:ignore
        return HeartBeatMsg(
            msg["Controller_Flags"],
            msg["Encoder_Flags"],
            msg["Motor_Flags"],
            msg["Axis_State"],
            msg["Axis_Error"],
        )

    def get_bus_voltage(self, axis_id) -> float:
        """Get the bus voltage.

        :raises: FunctionTimedOut
        """
        data = self._db.encode_message("Get_Vbus_Voltage")
        self.send(axis_id, ArbitrationCommand.GET_VBUS_VOLTAGE, data)

        rx = self.receive(
            axis_id, self._db.get_message_by_name("Get_Vbus_Voltage").frame_id
        )
        msg = self._db.decode_message("Get_Vbus_Voltage", rx.data)  # type:ignore
        return msg["Vbus_Voltage"]

    def get_axis_error(self, axis_id) -> float:
        """Get axis errors from the motor, if any.

        :raises: FunctionTimedOut
        """

        data = self._db.encode_message("Get_Motor_Error")
        self.send(axis_id, ArbitrationCommand.GET_MOTOR_ERROR, data)

        rx = self.receive(
            axis_id, self._db.get_message_by_name("Get_Motor_Error").frame_id
        )
        msg = self._db.decode_message("Get_Motor_Error", rx.data)  # type:ignore
        return msg
