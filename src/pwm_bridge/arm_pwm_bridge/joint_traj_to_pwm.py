from typing import Dict, List

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from mavros_msgs.msg import OverrideRCIn
from std_msgs.msg import Bool


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class JointTrajectoryToPWM(Node):
    """
    ROS2 node to convert JointTrajectory -> RC Override
    """

    def __init__(self):
        super().__init__("joint_traj_to_pwm")

        # ===== PARAMETERS =====
        self.declare_parameter("trajectory_topic", "arm_controller/joint_trajectory")
        self.declare_parameter("joint_names", ["joint1", "joint2", "joint3", "joint4", "joint5"])
        self.declare_parameter("rc_channels", [12, 13, 14, 15, 16])
        self.declare_parameter("rc_override_topic", "/uas1/mavros/rc/override")
        self.declare_parameter("require_start_trigger", False)
        self.declare_parameter("start_trigger_topic", "/arm_pwm_bridge/start")
        self.declare_parameter("pulse_min_us", 700.0)
        self.declare_parameter("pulse_max_us", 2300.0)
        self.declare_parameter("angle_min_rad", [-2.6, -2.0, -2.6, -2.6, -1.5])
        self.declare_parameter("angle_max_rad", [2.6, 2.6, 2.6, 2.6, 1.5])
        self.declare_parameter("initial_positions_rad", [0.0, 0.0, 0.0, 0.0, 0.0])

        # ===== LOAD PARAMETERS =====
        self.trajectory_topic: str = self.get_parameter("trajectory_topic").value
        self.joint_names: List[str] = list(self.get_parameter("joint_names").value)
        self.rc_channels: List[int] = list(self.get_parameter("rc_channels").value)
        self.rc_override_topic: str = self.get_parameter("rc_override_topic").value
        self.require_start_trigger: bool = bool(self.get_parameter("require_start_trigger").value)
        self.start_trigger_topic: str = self.get_parameter("start_trigger_topic").value
        self.pulse_min_us: float = self.get_parameter("pulse_min_us").value
        self.pulse_max_us: float = self.get_parameter("pulse_max_us").value
        self.angle_min_rad: List[float] = list(self.get_parameter("angle_min_rad").value)
        self.angle_max_rad: List[float] = list(self.get_parameter("angle_max_rad").value)
        self.initial_positions_rad: List[float] = list(self.get_parameter("initial_positions_rad").value)
        self._sending_enabled = not self.require_start_trigger

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

        self._invalid_limit_indices_logged = set()

        # ===== RC Override Publisher =====
        self.rc_pub = self.create_publisher(
            OverrideRCIn,
            self.rc_override_topic,
            10
        )

        self.rc_msg = OverrideRCIn()
        self.rc_msg.channels = [0] * 18

        # Store last joint positions
        self._last_positions: Dict[str, float] = {
            name: self.initial_positions_rad[i] if i < len(self.initial_positions_rad) else 0.0
            for i, name in enumerate(self.joint_names)
        }

        # Subscription
        self.traj_sub = self.create_subscription(
            JointTrajectory,
            self.trajectory_topic,
            self._traj_cb,
            10
        )
        self.start_trigger_sub = None
        if self.require_start_trigger:
            self.start_trigger_sub = self.create_subscription(
                Bool,
                self.start_trigger_topic,
                self._start_trigger_cb,
                10
            )

        self._rc_subscriber_warned = False
        self._rc_subscriber_connected = False
        self.subscriber_check_timer = self.create_timer(1.0, self._check_rc_subscribers)

        self.get_logger().info(
            f"Listening on {self.trajectory_topic} -> {self.rc_override_topic} channels {self.rc_channels}"
        )
        if self.require_start_trigger:
            self.get_logger().info(
                f"Start trigger enabled; waiting for Bool(true) on {self.start_trigger_topic} before publishing RC overrides."
            )

        # Apply initial positions
        if self._sending_enabled:
            self._write_positions(self._last_positions)
        else:
            self.get_logger().info("Initial RC positions armed but not published until start trigger is received.")

    # ==========================================================
    # Joint Trajectory Callback
    # ==========================================================

    def _traj_cb(self, msg: JointTrajectory):
        if not msg.points:
            return
        if not self._sending_enabled:
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

    # ==========================================================
    # Angle → PWM Conversion
    # ==========================================================

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

    # ==========================================================
    # RC Channel Writer
    # ==========================================================

    def _set_rc_channel(self, channel: int, pulse_us: float):
        # RC channels are 1-indexed, array is 0-indexed
        index = channel - 1

        if 0 <= index < 18:
            self.rc_msg.channels[index] = int(pulse_us)

    # ==========================================================
    # Initialize Positions
    # ==========================================================

    def _write_positions(self, positions: Dict[str, float]):
        for i, joint in enumerate(self.joint_names):
            if i >= len(self.rc_channels):
                continue

            pulse = self._angle_to_pwm(i, positions[joint])
            self._set_rc_channel(self.rc_channels[i], pulse)

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

    def _start_trigger_cb(self, msg: Bool):
        if msg.data and not self._sending_enabled:
            self._sending_enabled = True
            self.get_logger().info("Start trigger received. RC override publishing enabled.")
            self._write_positions(self._last_positions)
            return

        if not msg.data and self._sending_enabled:
            self._sending_enabled = False
            self.get_logger().warn("Start trigger set to false. RC override publishing paused.")


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
