from dataclasses import dataclass
from typing import List, Sequence, Tuple


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


@dataclass
class LogitechMappingConfig:
    deadzone: float = 0.12
    position_velocity_mps: float = 0.30
    joint3_velocity_radps: float = 1.20
    joint4_velocity_radps: float = 1.20
    fixed_x: float = 1.27
    initial_y: float = 0.0
    initial_z: float = -2.44
    y_limits: Tuple[float, float] = (-1.50, 1.50)
    z_limits: Tuple[float, float] = (-3.20, -0.20)
    left_stick_x_axis: int = 0
    left_stick_y_axis: int = 1
    right_stick_joint3_axis: int = 3
    joint4_right_button: int = 5
    joint4_left_button: int = 4
    gripper_open_axis: int = 2
    gripper_close_axis: int = 5
    left_stick_x_scale: float = 1.0
    left_stick_y_scale: float = 1.0
    right_stick_joint3_scale: float = -1.0
    gripper_scale: float = -1.0
    trigger_released_value: float = 1.0
    trigger_pressed_value: float = -1.0


@dataclass
class LogitechCommandOutput:
    pose_x: float
    pose_y: float
    pose_z: float
    joint3_command: float
    joint4_command: float
    gripper_command: float


class LogitechInterpreter:
    def __init__(self, config: LogitechMappingConfig | None = None) -> None:
        self.config = config or LogitechMappingConfig()
        self.current_y = self.config.initial_y
        self.current_z = self.config.initial_z

    def _axis_value(self, axes: Sequence[float], index: int, scale: float) -> float:
        if 0 <= index < len(axes):
            value = float(axes[index]) * scale
            if abs(value) < self.config.deadzone:
                return 0.0
            return value
        return 0.0

    def _trigger_value(self, axes: Sequence[float], index: int) -> float:
        if 0 <= index < len(axes):
            span = self.config.trigger_released_value - self.config.trigger_pressed_value
            if abs(span) < 1.0e-6:
                return 0.0

            value = clamp(
                float(axes[index]),
                min(self.config.trigger_pressed_value, self.config.trigger_released_value),
                max(self.config.trigger_pressed_value, self.config.trigger_released_value),
            )
            normalized = (self.config.trigger_released_value - value) / span
            if abs(normalized) < self.config.deadzone:
                return 0.0
            return clamp(normalized, 0.0, 1.0)
        return 0.0

    def _button_value(self, buttons: Sequence[int], index: int) -> float:
        if 0 <= index < len(buttons) and buttons[index]:
            return 1.0
        return 0.0

    def step(self, axes: Sequence[float], buttons: Sequence[int], dt: float) -> LogitechCommandOutput:
        left_x = self._axis_value(axes, self.config.left_stick_x_axis, self.config.left_stick_x_scale)
        left_y = self._axis_value(axes, self.config.left_stick_y_axis, self.config.left_stick_y_scale)
        joint3_axis = self._axis_value(
            axes,
            self.config.right_stick_joint3_axis,
            self.config.right_stick_joint3_scale,
        )

        self.current_y = clamp(
            self.current_y + (left_x * self.config.position_velocity_mps * dt),
            self.config.y_limits[0],
            self.config.y_limits[1],
        )
        self.current_z = clamp(
            self.current_z + (left_y * self.config.position_velocity_mps * dt),
            self.config.z_limits[0],
            self.config.z_limits[1],
        )

        joint3_command = joint3_axis * self.config.joint3_velocity_radps
        joint4_command = (
            self._button_value(buttons, self.config.joint4_right_button)
            - self._button_value(buttons, self.config.joint4_left_button)
        ) * self.config.joint4_velocity_radps
        gripper_command = (
            self._trigger_value(axes, self.config.gripper_open_axis)
            - self._trigger_value(axes, self.config.gripper_close_axis)
        ) * self.config.gripper_scale

        return LogitechCommandOutput(
            pose_x=self.config.fixed_x,
            pose_y=self.current_y,
            pose_z=self.current_z,
            joint3_command=joint3_command,
            joint4_command=joint4_command,
            gripper_command=gripper_command,
        )

    def snapshot(self) -> LogitechCommandOutput:
        return LogitechCommandOutput(
            pose_x=self.config.fixed_x,
            pose_y=self.current_y,
            pose_z=self.current_z,
            joint3_command=0.0,
            joint4_command=0.0,
            gripper_command=0.0,
        )
