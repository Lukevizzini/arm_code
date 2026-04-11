import ast
from typing import Optional
from typing import Dict, List, Sequence

import rclpy
from mavros_msgs.msg import OverrideRCIn
from rclpy.node import Node
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def parse_sequence_parameter(value, cast, parameter_name: str) -> List:
    if isinstance(value, str):
        try:
            value = ast.literal_eval(value)
        except (SyntaxError, ValueError) as exc:
            raise ValueError(f"Parameter '{parameter_name}' could not be parsed as a sequence: {value}") from exc

    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        raise ValueError(f"Parameter '{parameter_name}' must be a sequence, got {type(value).__name__}.")

    return [cast(item) for item in value]


class JointTrajectoryToPWM(Node):
    """ROS2 node to convert JointTrajectory into MAVROS RC overrides."""

    def __init__(self):
        super().__init__("joint_traj_to_pwm")

        self.declare_parameter("trajectory_topic", "arm_controller/joint_trajectory")
        self.declare_parameter("joint_names", "['joint1','joint2','joint3','joint4']")
        self.declare_parameter("rc_channels", "[12,13,14,15]")
        self.declare_parameter("rc_override_topic", "/mavros/rc/override")
        self.declare_parameter("pulse_min_us", 700.0)
        self.declare_parameter("pulse_max_us", 2300.0)
        self.declare_parameter("angle_min_rad", "[-2.6,-2.6,-2.6,-2.6]")
        self.declare_parameter("angle_max_rad", "[2.6,2.6,2.6,2.6]")
        self.declare_parameter("initial_positions_rad", "[0.0,0.0,0.0,0.0]")

        self.declare_parameter("gripper_topic", "/gripper_open_close_command")
        self.declare_parameter("gripper_channel", 16)
        self.declare_parameter("gripper_command_min", -1.0)
        self.declare_parameter("gripper_command_max", 1.0)
        self.declare_parameter("gripper_pulse_min_us", 900.0)
        self.declare_parameter("gripper_pulse_max_us", 2100.0)
        self.declare_parameter("gripper_command_is_rate", True)
        self.declare_parameter("gripper_rate_pwm_per_sec", 200.0)
        self.declare_parameter("initial_gripper_pwm_us", 1500.0)

        self.trajectory_topic: str = str(self.get_parameter("trajectory_topic").value)
        self.joint_names: List[str] = parse_sequence_parameter(
            self.get_parameter("joint_names").value,
            str,
            "joint_names",
        )
        self.rc_channels: List[int] = parse_sequence_parameter(
            self.get_parameter("rc_channels").value,
            int,
            "rc_channels",
        )
        self.rc_override_topic: str = str(self.get_parameter("rc_override_topic").value)
        self.pulse_min_us: float = float(self.get_parameter("pulse_min_us").value)
        self.pulse_max_us: float = float(self.get_parameter("pulse_max_us").value)
        self.angle_min_rad: List[float] = parse_sequence_parameter(
            self.get_parameter("angle_min_rad").value,
            float,
            "angle_min_rad",
        )
        self.angle_max_rad: List[float] = parse_sequence_parameter(
            self.get_parameter("angle_max_rad").value,
            float,
            "angle_max_rad",
        )
        self.initial_positions_rad: List[float] = parse_sequence_parameter(
            self.get_parameter("initial_positions_rad").value,
            float,
            "initial_positions_rad",
        )

        self.gripper_topic: str = str(self.get_parameter("gripper_topic").value)
        self.gripper_channel: int = int(self.get_parameter("gripper_channel").value)
        self.gripper_command_min: float = float(self.get_parameter("gripper_command_min").value)
        self.gripper_command_max: float = float(self.get_parameter("gripper_command_max").value)
        self.gripper_pulse_min_us: float = float(self.get_parameter("gripper_pulse_min_us").value)
        self.gripper_pulse_max_us: float = float(self.get_parameter("gripper_pulse_max_us").value)
        self.gripper_command_is_rate: bool = bool(self.get_parameter("gripper_command_is_rate").value)
        self.gripper_rate_pwm_per_sec: float = float(self.get_parameter("gripper_rate_pwm_per_sec").value)
        self.initial_gripper_pwm_us: float = float(self.get_parameter("initial_gripper_pwm_us").value)

        if self.pulse_min_us > self.pulse_max_us:
            self.get_logger().warn(
                f"pulse_min_us ({self.pulse_min_us}) > pulse_max_us ({self.pulse_max_us}); swapping values."
            )
            self.pulse_min_us, self.pulse_max_us = self.pulse_max_us, self.pulse_min_us

        joint_count = min(
            len(self.joint_names),
            len(self.rc_channels),
            len(self.angle_min_rad),
            len(self.angle_max_rad),
        )
        if joint_count == 0:
            raise ValueError("joint_names, rc_channels, angle_min_rad, and angle_max_rad must be non-empty")

        if joint_count < len(self.joint_names):
            self.get_logger().warn(
                "Parameter length mismatch detected; truncating to the shortest list "
                f"({joint_count} joints/channels)."
            )

        self.joint_names = self.joint_names[:joint_count]
        self.rc_channels = self.rc_channels[:joint_count]
        self.angle_min_rad = self.angle_min_rad[:joint_count]
        self.angle_max_rad = self.angle_max_rad[:joint_count]

        if len(self.initial_positions_rad) < joint_count:
            self.initial_positions_rad.extend([0.0] * (joint_count - len(self.initial_positions_rad)))
        else:
            self.initial_positions_rad = self.initial_positions_rad[:joint_count]

        self.rc_pub = self.create_publisher(OverrideRCIn, self.rc_override_topic, 10)
        self.rc_msg = OverrideRCIn()
        self.rc_msg.channels = [0] * 18

        self._invalid_limit_indices_logged = set()
        self._rc_subscriber_warned = False
        self._rc_subscriber_connected = False
        self._last_gripper_command_time: Optional[float] = None

        self._last_positions: Dict[str, float] = {
            name: self.initial_positions_rad[i] if i < len(self.initial_positions_rad) else 0.0
            for i, name in enumerate(self.joint_names)
        }
        self._current_gripper_pwm_us = clamp(
            self.initial_gripper_pwm_us,
            min(self.gripper_pulse_min_us, self.gripper_pulse_max_us),
            max(self.gripper_pulse_min_us, self.gripper_pulse_max_us),
        )

        self.create_subscription(JointTrajectory, self.trajectory_topic, self._traj_cb, 10)
        if self.gripper_channel > 0:
            self.create_subscription(Float64, self.gripper_topic, self._gripper_cb, 10)

        self.subscriber_check_timer = self.create_timer(1.0, self._check_rc_subscribers)

        self.get_logger().info(
            f"Listening on {self.trajectory_topic} -> {self.rc_override_topic} channels {self.rc_channels}"
        )
        if self.gripper_channel > 0:
            self.get_logger().info(
                f"Listening on {self.gripper_topic} -> RC channel {self.gripper_channel}"
            )
            if self.gripper_command_is_rate:
                self.get_logger().info(
                    "Gripper commands are interpreted as a rate; zero input holds the last PWM value."
                )

        self._write_positions(self._last_positions)

    def _traj_cb(self, msg: JointTrajectory):
        if not msg.points:
            return

        point = msg.points[-1]
        name_to_index = {name: i for i, name in enumerate(msg.joint_names)}

        for i, joint in enumerate(self.joint_names):
            if i >= len(self.rc_channels):
                continue

            js_idx = name_to_index.get(joint)
            if js_idx is None or js_idx >= len(point.positions):
                continue

            angle = float(point.positions[js_idx])
            clamped = clamp(angle, self.angle_min_rad[i], self.angle_max_rad[i])
            pulse = self._angle_to_pwm(i, clamped)

            self._set_rc_channel(self.rc_channels[i], pulse)
            self._last_positions[joint] = clamped

        self.rc_pub.publish(self.rc_msg)

    def _gripper_cb(self, msg: Float64):
        if self.gripper_channel <= 0:
            return

        command = clamp(
            float(msg.data),
            min(self.gripper_command_min, self.gripper_command_max),
            max(self.gripper_command_min, self.gripper_command_max),
        )
        if self.gripper_command_is_rate:
            pulse = self._gripper_rate_to_pwm(command)
        else:
            pulse = self._gripper_to_pwm(command)
        self._set_rc_channel(self.gripper_channel, pulse)
        self.rc_pub.publish(self.rc_msg)

    def _angle_to_pwm(self, idx: int, angle: float) -> float:
        lower = self.angle_min_rad[idx]
        upper = self.angle_max_rad[idx]
        if upper <= lower:
            if idx not in self._invalid_limit_indices_logged:
                joint_name = self.joint_names[idx] if idx < len(self.joint_names) else f"joint_index_{idx}"
                self.get_logger().warn(
                    f"Invalid angle limits for {joint_name}: lower={lower}, upper={upper}. "
                    "Using pulse_min_us for this joint."
                )
                self._invalid_limit_indices_logged.add(idx)
            return self.pulse_min_us

        ratio = (angle - lower) / (upper - lower)
        pwm = self.pulse_min_us + ratio * (self.pulse_max_us - self.pulse_min_us)
        return clamp(pwm, self.pulse_min_us, self.pulse_max_us)

    def _gripper_to_pwm(self, command: float) -> float:
        lower = self.gripper_command_min
        upper = self.gripper_command_max
        span = upper - lower
        if abs(span) < 1.0e-6:
            return self.gripper_pulse_min_us

        ratio = (command - lower) / span
        pwm = self.gripper_pulse_min_us + ratio * (self.gripper_pulse_max_us - self.gripper_pulse_min_us)
        return clamp(
            pwm,
            min(self.gripper_pulse_min_us, self.gripper_pulse_max_us),
            max(self.gripper_pulse_min_us, self.gripper_pulse_max_us),
        )

    def _gripper_rate_to_pwm(self, command: float) -> float:
        now_sec = self.get_clock().now().nanoseconds / 1.0e9
        lower = min(self.gripper_pulse_min_us, self.gripper_pulse_max_us)
        upper = max(self.gripper_pulse_min_us, self.gripper_pulse_max_us)

        if self._last_gripper_command_time is None:
            self._last_gripper_command_time = now_sec
            return self._current_gripper_pwm_us

        dt = max(0.0, now_sec - self._last_gripper_command_time)
        self._last_gripper_command_time = now_sec

        self._current_gripper_pwm_us = clamp(
            self._current_gripper_pwm_us + (command * self.gripper_rate_pwm_per_sec * dt),
            lower,
            upper,
        )
        return self._current_gripper_pwm_us

    def _set_rc_channel(self, channel: int, pulse_us: float):
        index = channel - 1
        if 0 <= index < 18:
            self.rc_msg.channels[index] = int(pulse_us)

    def _write_positions(self, positions: Dict[str, float]):
        for i, joint in enumerate(self.joint_names):
            if i >= len(self.rc_channels):
                continue

            pulse = self._angle_to_pwm(i, positions[joint])
            self._set_rc_channel(self.rc_channels[i], pulse)

        if self.gripper_channel > 0:
            self._set_rc_channel(self.gripper_channel, self._current_gripper_pwm_us)

        self.rc_pub.publish(self.rc_msg)
        self.get_logger().info("Initial RC override positions applied")

    def _check_rc_subscribers(self):
        subscription_count = self.rc_pub.get_subscription_count()
        if subscription_count > 0:
            if not self._rc_subscriber_connected:
                self.get_logger().info(
                    f"RC override topic has {subscription_count} subscriber(s)."
                )
                self._rc_subscriber_connected = True
            return

        if not self._rc_subscriber_warned:
            self.get_logger().warn(
                f"No subscribers on {self.rc_override_topic}; RC overrides will not reach the FCU. "
                "Start MAVROS or verify the topic namespace."
            )
            self._rc_subscriber_warned = True


def main(args=None):
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
