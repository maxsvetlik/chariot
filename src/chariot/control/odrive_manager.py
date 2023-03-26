import dataclasses
import enum
from typing import Dict, Sequence, Tuple

from .can_simple_interface import CanProtocolSettings, CanSimpleInterface, HeartBeatMsg


@dataclasses.dataclass()
class ChariotProtocolSettings(CanProtocolSettings):
    device_id: str = "can0"
    bitrate: int = "500000"


@dataclasses.dataclass(frozen=True)
class CanMotorLayout:
    """Designates the CAN node ID as a relative layout."""

    left_rear_id: bytes = 0x0
    right_rear_id: bytes = 0x1
    right_front_id: bytes = 0x2
    left_front_id: bytes = 0x3

    @property
    def axis_ids(self) -> Sequence[bytes]:
        return [
            self.left_rear_id,
            self.right_rear_id,
            self.right_front_id,
            self.left_front_id,
        ]

    @property
    def left_ids(self) -> Sequence[bytes]:
        return [self.left_rear_id, self.left_front_id]

    @property
    def right_ids(self) -> Sequence[bytes]:
        return [self.right_rear_id, self.right_front_id]


class RotationMasks(enum.Enum):
    """Designates the absolute direction of each motor for a relative movement.
    The layout of each mask is [LeftRear, RightRear, RearFront, LeftFront]."""

    FORWARD = [-1, 1, 1, 1]


@dataclasses.dataclass(frozen=True)
class ChariotKinematics:
    can_motor_layout: CanMotorLayout = CanMotorLayout()
    wheel_gear_ratio: int = 20
    wheel_diameter: float = 0.254  # meters
    wheel_rotation_mask = [-1, 1, 1, -1]
    wheel_base: float = 0.4826  # meters

    @property
    def wheel_radius(self) -> float:
        return self.wheel_diameter / 2


class OdriveMotorManager:
    """This is the interface layer between client applications and the lower level CAN interface. In general, if
    interaction with motors is required, it should go through the motor manager.

    It is expected that the CAN interface be up on the Operating System level. The CAN bus ID should be reflected
    in the CanProtocolSettings object passed in.

    :param can_protocol_settings: the specification for connecting to the CAN bus.
    """

    _kinematics : ChariotKinematics = ChariotKinematics()
    _axis_ids : Sequence[int] = _kinematics.can_motor_layout.axis_ids

    def __init__(
        self, can_protocol_settings: CanProtocolSettings = ChariotProtocolSettings
    ) -> None:

        self._can_interface = CanSimpleInterface(protocol_settings=can_protocol_settings)
        

    def set_motors_active(self) -> None:
        """On the first motor transition from IDLE to CLOSED LOOP CONTROL, we must also set the controller and
        controller profile, if any. For convenience this is set on every state transition."""
        self.set_motors_idle()
        [self._can_interface.set_velocity_control_mode(id) for id in self._axis_ids]
        [self._can_interface.set_axis_closed_loop(id) for id in self._axis_ids]

    def set_motors_idle(self) -> None:
        """Set all motors to the IDLE state."""
        [self._can_interface.set_axis_idle(id) for id in self._axis_ids]

    def set_motors_linear_velocity(self, velocities: Dict[int, float]) -> None:
        """Set target velocity for each motor as a mapping from CAN node ID to velocity."""

        assert (
            velocities.keys in self._axis_ids
        ), "Requested velocity for a motor ID that does not exist."
        for id, velocity in velocities:
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
        for _, id in enumerate(CanMotorLayout().left_ids):
            self._can_interface.set_target_velocity(
                id,
                vel_left
                * self._kinematics.wheel_rotation_mask[
                    self._kinematics.can_motor_layout.axis_ids.index(id)
                ],
            )

        for _, id in enumerate(CanMotorLayout().right_ids):
            self._can_interface.set_target_velocity(
                id,
                vel_right
                * self._kinematics.wheel_rotation_mask[
                    self._kinematics.can_motor_layout.axis_ids.index(id)
                ],
            )

    def dump_motor_errors(self) -> Sequence[float]:
        """Return the motor error status for each motor."""
        return [self._can_interface.get_axis_error(id) for id in self._axis_ids]

    def dump_heartbeat(self) -> Sequence[Tuple[int, HeartBeatMsg]]:
        """Return the heartbeat message for each motor."""
        return [(id, self._can_interface.get_heartbeat(id)) for id in self._axis_ids]

    def clear_motor_errors(self) -> None:
        [self._can_interface.clear_motor_error(id) for id in self._axis_ids]
