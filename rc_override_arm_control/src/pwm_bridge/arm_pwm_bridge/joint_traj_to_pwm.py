import math
from typing import Dict, List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Joy
from trajectory_msgs.msg import JointTrajectory

try:
    from mavros_msgs.msg import OverrideRCIn  # type: ignore
except ImportError:  # pragma: no cover - runtime dependency, not present in CI by default
    OverrideRCIn = None


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class JointTrajectoryToPWM(Node):
    """
    Bridge: listens to JointTrajectory and drives Navigator PWM channels via
    MAVROS RC override messages. Optional gripper button control can be fed
    directly from Joy messages.
    """

    def __init__(self) -> None:
        super().__init__("joint_traj_to_pwm")

        self.declare_parameter("trajectory_topic", "arm_controller/joint_trajectory")
        self.declare_parameter("joint_names", ["joint1", "joint2", "joint3", "joint4", "gripper_joint"])
        self.declare_parameter("pwm_channels", [11, 12, 13, 14, 15])
        self.declare_parameter("pulse_min_us", 700.0)
        self.declare_parameter("pulse_max_us", 2300.0)
        self.declare_parameter("angle_min_rad", [-2.6, -2.0, -2.6, -2.6, -1.0])
        self.declare_parameter("angle_max_rad", [2.6, 2.6, 2.6, 2.6, 1.0])
        self.declare_parameter("initial_positions_rad", [0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("gripper_joy_topic", "joy")
        self.declare_parameter("gripper_joint_name", "gripper_joint")
        self.declare_parameter("gripper_open_button_index", 5)   # RB / "o"
        self.declare_parameter("gripper_close_button_index", 4)  # LB / "u"
        self.declare_parameter("gripper_step_rad", 0.08)
        self.declare_parameter("override_topic", "/mavros/rc/override")
        self.declare_parameter("send_neutral_on_shutdown", True)
        self.declare_parameter("neutral_pwm", 1500)
        self.declare_parameter("publish_joint_states", True)
        self.declare_parameter("joint_state_rate_hz", 10.0)

        self.trajectory_topic = str(self.get_parameter("trajectory_topic").value)
        self.joint_names: List[str] = list(self.get_parameter("joint_names").value)
        self.pwm_channels: List[int] = [int(ch) for ch in list(self.get_parameter("pwm_channels").value)]
        self.pulse_min_us: float = float(self.get_parameter("pulse_min_us").value)
        self.pulse_max_us: float = float(self.get_parameter("pulse_max_us").value)
        self.angle_min_rad: List[float] = list(self.get_parameter("angle_min_rad").value)
        self.angle_max_rad: List[float] = list(self.get_parameter("angle_max_rad").value)
        self.initial_positions_rad: List[float] = list(self.get_parameter("initial_positions_rad").value)
        self.gripper_joy_topic: str = str(self.get_parameter("gripper_joy_topic").value)
        self.gripper_joint_name: str = str(self.get_parameter("gripper_joint_name").value)
        self.gripper_open_button_index: int = int(self.get_parameter("gripper_open_button_index").value)
        self.gripper_close_button_index: int = int(self.get_parameter("gripper_close_button_index").value)
        self.gripper_step_rad: float = float(self.get_parameter("gripper_step_rad").value)
        self.override_topic: str = str(self.get_parameter("override_topic").value).strip()
        self.send_neutral_on_shutdown: bool = bool(self.get_parameter("send_neutral_on_shutdown").value)
        self.neutral_pwm: int = int(self.get_parameter("neutral_pwm").value)
        self.publish_joint_states: bool = bool(self.get_parameter("publish_joint_states").value)
        self.joint_state_rate_hz: float = float(self.get_parameter("joint_state_rate_hz").value)

        if len(self.joint_names) != len(self.pwm_channels):
            self.get_logger().warn(
                f"joint_names ({len(self.joint_names)}) and pwm_channels ({len(self.pwm_channels)}) length mismatch; "
                "extra joints or channels will be ignored."
            )

        self._rc_pub = None
        self._override_channel_count = 0
        self._mavros_ready = False
        self._warned_disabled = False
        self._warned_out_of_range_channels: List[int] = []
        self._last_positions: Dict[str, float] = {
            name: self._value_for_idx(self.initial_positions_rad, idx, 0.0)
            for idx, name in enumerate(self.joint_names)
        }
        self._gripper_joint_idx = self._resolve_gripper_joint_idx()

        self._connect_override_pub()

        self.create_subscription(JointTrajectory, self.trajectory_topic, self._traj_cb, 10)
        self.get_logger().info(
            f"Listening on {self.trajectory_topic} for JointTrajectory commands -> MAVROS RC override."
        )

        if self._gripper_joint_idx >= 0:
            self.create_subscription(Joy, self.gripper_joy_topic, self._joy_cb, 10)
            self.get_logger().info(
                f"Gripper control enabled from {self.gripper_joy_topic}: "
                f"close button {self.gripper_close_button_index}, "
                f"open button {self.gripper_open_button_index}, "
                f"channel {self.pwm_channels[self._gripper_joint_idx]}."
            )

        if self.publish_joint_states:
            self._js_pub = self.create_publisher(JointState, "joint_states", 10)
            self.create_timer(1.0 / self.joint_state_rate_hz, self._publish_joint_states)
        else:
            self._js_pub = None

        # Apply initial positions if specified.
        if self._mavros_ready:
            self._write_positions(self._last_positions, log=False)

    def _value_for_idx(self, values: List[float], idx: int, default: float) -> float:
        return float(values[idx]) if idx < len(values) else default

    def _resolve_gripper_joint_idx(self) -> int:
        if self.gripper_joint_name not in self.joint_names:
            self.get_logger().warn(
                f"gripper_joint_name '{self.gripper_joint_name}' is not in joint_names; "
                "gripper button control is disabled."
            )
            return -1
        return self.joint_names.index(self.gripper_joint_name)

    def _button_pressed(self, msg: Joy, index: int) -> bool:
        return 0 <= index < len(msg.buttons) and bool(msg.buttons[index])

    def _joy_cb(self, msg: Joy) -> None:
        if self._gripper_joint_idx < 0:
            return

        open_pressed = self._button_pressed(msg, self.gripper_open_button_index)
        close_pressed = self._button_pressed(msg, self.gripper_close_button_index)

        direction = 0
        if open_pressed and not close_pressed:
            direction = 1
        elif close_pressed and not open_pressed:
            direction = -1

        if direction == 0:
            return

        joint = self.joint_names[self._gripper_joint_idx]
        current = self._last_positions.get(joint, self._value_for_idx(self.initial_positions_rad, self._gripper_joint_idx, 0.0))
        updated = self._clamp_angle(self._gripper_joint_idx, current + (direction * self.gripper_step_rad))
        if abs(updated - current) < 1e-9:
            return

        self._last_positions[joint] = updated
        self._publish_override(self._last_positions)

    def _connect_override_pub(self) -> None:
        if OverrideRCIn is None:
            self.get_logger().error(
                "mavros_msgs is not available. Install ROS mavros messages package to enable MAVROS PWM output."
            )
            return

        if not self.override_topic:
            self.get_logger().error(
                "override_topic is empty. Set it to a valid topic, e.g. /mavros/rc/override."
            )
            return

        self._rc_pub = self.create_publisher(OverrideRCIn, self.override_topic, 10)  # type: ignore[arg-type]
        template = OverrideRCIn()
        self._override_channel_count = len(template.channels)
        if self._override_channel_count <= 0:
            self._override_channel_count = 18  # MAVROS OverrideRCIn channel count
        for channel in self.pwm_channels:
            if channel <= 0:
                self.get_logger().warn(
                    f"Invalid pwm channel {channel}; channels should be positive (Navigator: 1-16)."
                )

        self._mavros_ready = True
        self.get_logger().info(
            f"MAVROS RC override publisher ready on {self.override_topic}; "
            f"driving channels {self.pwm_channels}."
        )

    def _traj_cb(self, msg: JointTrajectory) -> None:
        if not msg.points:
            return
        if not self._mavros_ready or self._rc_pub is None:
            if not self._warned_disabled:
                self.get_logger().warn("PWM disabled (MAVROS RC override publisher not ready); dropping messages.")
                self._warned_disabled = True
            return

        point = msg.points[-1]
        name_to_index = {name: idx for idx, name in enumerate(msg.joint_names)}
        updates: Dict[str, float] = {}

        for idx, joint in enumerate(self.joint_names):
            if idx >= len(self.pwm_channels):
                continue
            channel = self.pwm_channels[idx]
            js_idx = name_to_index.get(joint)
            if js_idx is None:
                continue
            if js_idx >= len(point.positions):
                continue

            angle = float(point.positions[js_idx])
            clamped = self._clamp_angle(idx, angle)
            updates[joint] = clamped

        if updates:
            self._last_positions.update(updates)
            self._publish_override(self._last_positions)

    def _clamp_angle(self, idx: int, angle: float) -> float:
        lower = self._value_for_idx(self.angle_min_rad, idx, -math.pi)
        upper = self._value_for_idx(self.angle_max_rad, idx, math.pi)
        if upper <= lower:
            return angle
        return clamp(angle, lower, upper)

    def _angle_to_pulse(self, idx: int, angle: float) -> float:
        lower = self._value_for_idx(self.angle_min_rad, idx, -math.pi)
        upper = self._value_for_idx(self.angle_max_rad, idx, math.pi)
        pulse_min = self.pulse_min_us
        pulse_max = self.pulse_max_us
        if upper <= lower or pulse_max <= pulse_min:
            return pulse_min
        ratio = (angle - lower) / (upper - lower)
        return clamp(pulse_min + ratio * (pulse_max - pulse_min), pulse_min, pulse_max)

    def _publish_joint_states(self) -> None:
        if not self._js_pub:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joint_names)
        msg.position = [self._last_positions.get(name, 0.0) for name in self.joint_names]
        self._js_pub.publish(msg)

    def _publish_override(self, positions: Dict[str, float]) -> None:
        if not self._mavros_ready or self._rc_pub is None or OverrideRCIn is None:
            return

        msg = OverrideRCIn()
        nochange = OverrideRCIn.CHAN_NOCHANGE
        msg.channels = [nochange] * self._override_channel_count

        for idx, joint in enumerate(self.joint_names):
            if idx >= len(self.pwm_channels):
                continue
            channel = self.pwm_channels[idx]
            if channel <= 0:
                continue
            array_idx = channel - 1
            if array_idx >= self._override_channel_count:
                if channel not in self._warned_out_of_range_channels:
                    self.get_logger().warn(
                        f"PWM channel {channel} is out of range for OverrideRCIn "
                        f"(max {self._override_channel_count}); skipping."
                    )
                    self._warned_out_of_range_channels.append(channel)
                continue
            angle = positions.get(joint, 0.0)
            clamped = self._clamp_angle(idx, angle)
            pulse_us = self._angle_to_pulse(idx, clamped)
            msg.channels[array_idx] = int(round(pulse_us))

        self._rc_pub.publish(msg)

    def _write_positions(self, positions: Dict[str, float], log: bool = True) -> None:
        if not self._mavros_ready:
            return
        self._publish_override(positions)
        if log:
            self.get_logger().info(f"Applied initial positions (us in [{self.pulse_min_us}, {self.pulse_max_us}]).")

    def destroy_node(self) -> bool:
        if self._mavros_ready and self._rc_pub and OverrideRCIn is not None and self.send_neutral_on_shutdown:
            msg = OverrideRCIn()
            nochange = OverrideRCIn.CHAN_NOCHANGE
            msg.channels = [nochange] * self._override_channel_count
            for channel in self.pwm_channels:
                if channel <= 0:
                    continue
                array_idx = channel - 1
                if array_idx < self._override_channel_count:
                    msg.channels[array_idx] = int(self.neutral_pwm)
            self._rc_pub.publish(msg)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JointTrajectoryToPWM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
