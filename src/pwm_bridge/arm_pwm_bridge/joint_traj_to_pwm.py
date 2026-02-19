import math
from typing import Dict, List

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from mavros_msgs.msg import OverrideRCIn


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class JointTrajectoryToPWM(Node):
    """
    ROS2 node to convert JointTrajectory -> RC Override (channels 11-15)
    """

    def __init__(self):
        super().__init__("joint_traj_to_rc_override")

        # ===== PARAMETERS =====
        self.declare_parameter("trajectory_topic", "/arm_controller/joint_trajectory")
        self.declare_parameter("joint_names", ["joint1", "joint2", "joint3", "joint4", "joint5"])
        self.declare_parameter("rc_channels", [11, 12, 13, 14, 15])
        self.declare_parameter("pulse_min_us", 700.0)
        self.declare_parameter("pulse_max_us", 2300.0)
        self.declare_parameter("angle_min_rad", [-2.6, -2.0, -2.6, -2.6, -1.5])
        self.declare_parameter("angle_max_rad", [2.6, 2.6, 2.6, 2.6, 1.5])
        self.declare_parameter("initial_positions_rad", [0.0, 0.0, 0.0, 0.0, 0.0])

        # ===== LOAD PARAMETERS =====
        self.trajectory_topic: str = self.get_parameter("trajectory_topic").value
        self.joint_names: List[str] = self.get_parameter("joint_names").value
        self.rc_channels: List[int] = self.get_parameter("rc_channels").value
        self.pulse_min_us: float = self.get_parameter("pulse_min_us").value
        self.pulse_max_us: float = self.get_parameter("pulse_max_us").value
        self.angle_min_rad: List[float] = self.get_parameter("angle_min_rad").value
        self.angle_max_rad: List[float] = self.get_parameter("angle_max_rad").value
        self.initial_positions_rad: List[float] = self.get_parameter("initial_positions_rad").value

        # ===== RC Override Publisher =====
        self.rc_pub = self.create_publisher(
            OverrideRCIn,
            "mavros/rc/override",
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
        self.create_subscription(
            JointTrajectory,
            self.trajectory_topic,
            self._traj_cb,
            10
        )

        self.get_logger().info(
            f"Listening on {self.trajectory_topic} -> RC Override {self.rc_channels}"
        )

        # Apply initial positions
        self._write_positions(self._last_positions)

    # ==========================================================
    # Joint Trajectory Callback
    # ==========================================================

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

    # ==========================================================
    # Angle → PWM Conversion
    # ==========================================================

    def _angle_to_pwm(self, idx: int, angle: float) -> float:
        lower = self.angle_min_rad[idx]
        upper = self.angle_max_rad[idx]

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