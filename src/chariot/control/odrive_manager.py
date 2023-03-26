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

import dataclasses
import enum
from typing import Dict, Sequence, Tuple, List

from .can_simple_interface import (
    CanProtocolSettings,
    CanSimpleInterface,
    HeartBeatMsg,
    EncoderEstimateMsg,
)


@dataclasses.dataclass()
class ChariotProtocolSettings(CanProtocolSettings):
    device_id: str = "can0"
    bitrate: str = "500000"


@dataclasses.dataclass(frozen=True)
class CanMotorLayout:
    """Designates the CAN node ID as a relative layout."""

    left_rear_id: int = 0x0
    right_rear_id: int = 0x1
    right_front_id: int = 0x2
    left_front_id: int = 0x3

    @property
    def axis_ids(self) -> Sequence[int]:
        return [
            self.left_rear_id,
            self.right_rear_id,
            self.right_front_id,
            self.left_front_id,
        ]

    @property
    def left_ids(self) -> Sequence[int]:
        return [self.left_rear_id, self.left_front_id]

    @property
    def right_ids(self) -> Sequence[int]:
        return [self.right_rear_id, self.right_front_id]


@dataclasses.dataclass(frozen=True)
class ChariotKinematics:
    can_motor_layout: CanMotorLayout = CanMotorLayout()
    wheel_gear_ratio: int = 20
    wheel_diameter: float = 0.254  # meters
    wheel_rotation_mask = [
        -1,
        1,
        1,
        -1,
    ]  # Designates the absolute direction of each motor for a relative movement
    # The layout is [LeftRear, RightRear, RearFront, LeftFront]
    wheel_base: float = 0.4826  # meters
    mps_per_rotation: float = 0.0398 # m/s velocity for every unit increase in motor rot/s accounting for the gear ratio

    @property
    def wheel_radius(self) -> float:
        return self.wheel_diameter / 2


@dataclasses.dataclass(frozen=True)
class ErrorInfo:
    controller_flags: str  # Errors on the control level
    encoder_flags: str  # Errors specific to the encoder
    motor_flags: str  # Errors specific to the motor
    axis_error: str  # Errors specific to the axis control level


@dataclasses.dataclass(frozen=True)
class MotorInfo:
    axis_id: int  # The CAN node ID of the motor
    axis_state: str  # The state the axis is in
    pos_estimate: float  # Position estimates using the encoder
    vel_estimate: float  # Velocity estimates using the encoder
    voltage: float  # The controller's VBus voltage
    error_info: ErrorInfo  # Error states for components


class OdriveMotorManager:
    """This is the interface layer between client applications and the lower level CAN interface. In general, if
    interaction with motors is required, it should go through the motor manager.

    It is expected that the CAN interface be up on the Operating System level. The CAN bus ID should be reflected
    in the CanProtocolSettings object passed in.

    :param can_protocol_settings: the specification for connecting to the CAN bus.
    """

    _kinematics: ChariotKinematics = ChariotKinematics()
    _axis_ids: Sequence[int] = _kinematics.can_motor_layout.axis_ids

    def __init__(
        self, can_protocol_settings: CanProtocolSettings = ChariotProtocolSettings()
    ) -> None:
        self._can_interface = CanSimpleInterface(
            protocol_settings=can_protocol_settings
        )

    def set_motors_active(self) -> None:
        """On the first motor transition from IDLE to CLOSED LOOP CONTROL, we must also set the controller and
        controller profile, if any. For convenience this is set on every state transition.
        """
        self.set_motors_idle()  # Ensure the motors are in a state where modes can be changed
        self.clear_errors()  # N.B. Certain errors are not cleared automatically, like WATCHDOG_TIMER_EXPIRED even
        # after the watchdog is reset. Clearing the errors here sets the error flags in a clean state.
        [self._can_interface.set_velocity_control_mode(id) for id in self._axis_ids]
        [self._can_interface.set_axis_closed_loop(id) for id in self._axis_ids]

    def set_motors_idle(self) -> None:
        """Set all motors to the IDLE state."""
        [self._can_interface.set_axis_idle(id) for id in self._axis_ids]

    def set_motors_linear_velocity(self, id: int, velocity: float) -> None:
        """Set target velocity for a given motor.

        :param id: the motor ID to command
        :param velocity: the desired velocity in rotations / s
        """

        self._can_interface.set_target_velocity(
            id,
            velocity
            * self._kinematics.wheel_rotation_mask[
                self._kinematics.can_motor_layout.axis_ids.index(id)
            ],
        )

    def set_motors_arc_velocity(
        self, linear_velocity: float, angular_velocity: float
    ) -> None:
        """A driving model for differential drive wherein the target wheel speed is calculated from the desired
        velocity.
        """
        length = self._kinematics.wheel_base
        vel_left = (
            linear_velocity - (length / 2) * angular_velocity
        ) / self._kinematics.wheel_radius
        vel_right = (
            linear_velocity + (length / 2) * angular_velocity
        ) / self._kinematics.wheel_radius

        # Set the target velocities to the appropriate wheels and apply a mask to correct for motor orientation
        for _, id in enumerate(self._kinematics.can_motor_layout.left_ids):
            self._can_interface.set_target_velocity(
                id,
                vel_left
                * self._kinematics.wheel_rotation_mask[
                    self._kinematics.can_motor_layout.axis_ids.index(id)
                ],
            )

        for _, id in enumerate(self._kinematics.can_motor_layout.right_ids):
            self._can_interface.set_target_velocity(
                id,
                vel_right
                * self._kinematics.wheel_rotation_mask[
                    self._kinematics.can_motor_layout.axis_ids.index(id)
                ],
            )

    def get_motor_error(self) -> Sequence[str]:
        """Return the motor error status for each motor."""
        return [self._can_interface.get_motor_error(id) for id in self._axis_ids]

    def get_encoder_error(self) -> Sequence[str]:
        return [self._can_interface.get_encoder_error(id) for id in self._axis_ids]

    def get_heartbeat(self) -> Sequence[HeartBeatMsg]:
        """Return the heartbeat message for each motor."""
        return [self._can_interface.get_heartbeat(id) for id in self._axis_ids]

    def clear_errors(self) -> None:
        [
            self._can_interface.clear_motor_error(id) for id in self._axis_ids
        ]  # type:ignore

    def get_vbus_voltage(self) -> Sequence[float]:
        return [self._can_interface.get_bus_voltage(id) for id in self._axis_ids]

    def get_encoder_estimate(self) -> Sequence[EncoderEstimateMsg]:
        return [self._can_interface.get_encoder_estimate(id) for id in self._axis_ids]

    def get_motor_info(self) -> Sequence[MotorInfo]:
        heartbeats = self.get_heartbeat()
        estimates = self.get_encoder_estimate()
        voltages = self.get_vbus_voltage()

        infos: List[MotorInfo] = []
        for id in self._axis_ids:
            heartbeat = heartbeats[id]
            estimate = estimates[id]
            voltage = voltages[id]
            err = ErrorInfo(
                heartbeat.controller_flags,
                heartbeat.encoder_flags,
                heartbeat.motor_flags,
                heartbeat.axis_error,
            )
            infos.append(
                MotorInfo(
                    id,
                    heartbeat.axis_state,
                    estimate.pos_estiamte,
                    estimate.vel_estimate,
                    voltage,
                    err,
                )
            )
        return infos
