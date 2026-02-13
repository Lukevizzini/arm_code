import math
from typing import Dict,List, Set

import rclpy
from rclpy.node import Node
from rclpy.task import Future
from trajectory_msgs.msg import JointTrajectory

from mavros_msgs.srv import CommandLong

MAV_CMD_DO_SET_SERVO = 183


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class JointTrajectoryToPWM(Node):
    """ROS2 node to convert JointTrajectory -> PWM11-15 via MAV_CMD_DO_SET_SERVO"""

    def __init__(self):
        super().__init__("joint_traj_to_pwm_11_15")

        # ===== PARAMETERS =====
        self.declare_parameter("trajectory_topic", "/arm_controller/joint_trajectory")
        self.declare_parameter("joint_names", ["joint1", "joint2", "joint3", "joint4", "joint5"])
        self.declare_parameter("pwm_channels", [11, 12, 13, 14, 15])
        self.declare_parameter("pulse_min_us", 700.0)
        self.declare_parameter("pulse_max_us", 2300.0)
        self.declare_parameter("angle_min_rad", [-2.6, -2.0, -2.6, -2.6, -1.5])
        self.declare_parameter("angle_max_rad", [2.6, 2.6, 2.6, 2.6, 1.5])
        self.declare_parameter("initial_positions_rad", [0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("command_service", "/mavros/cmd/command")
        self.declare_parameter("wait_for_service", True)
        self.declare_parameter("service_timeout_sec", 5.0)

        # ===== LOAD PARAMETERS =====
        self.trajectory_topic: str = self.get_parameter("trajectory_topic").value
        self.joint_names: List[str] = self.get_parameter("joint_names").value
        self.pwm_channels: List[int] = self.get_parameter("pwm_channels").value
        self.pulse_min_us: float = self.get_parameter("pulse_min_us").value
        self.pulse_max_us: float = self.get_parameter("pulse_max_us").value
        self.angle_min_rad: List[float] = self.get_parameter("angle_min_rad").value
        self.angle_max_rad: List[float] = self.get_parameter("angle_max_rad").value
        self.initial_positions_rad: List[float] = self.get_parameter("initial_positions_rad").value
        self.command_service: str = self.get_parameter("command_service").value
        self.wait_for_service: bool = self.get_parameter("wait_for_service").value
        self.service_timeout_sec: float = self.get_parameter("service_timeout_sec").value

        self._cmd_client = None
        self._mavros_ready = False
        self._pending_calls: Set[Future] = set()
        self._last_positions: Dict[str, float] = {
            name: self.initial_positions_rad[i] if i < len(self.initial_positions_rad) else 0.0
            for i, name in enumerate(self.joint_names)
        }

        self._connect_mavros()

        self.create_subscription(
            JointTrajectory,
            self.trajectory_topic,
            self._traj_cb,
            10
        )
        self.get_logger().info(
            f"Listening on {self.trajectory_topic} -> PWM {self.pwm_channels}"
        )

        self._write_positions(self._last_positions, log=False)

    def _connect_mavros(self):
        if CommandLong is None:
            self.get_logger().error("mavros_msgs not installed")
            return

        self._cmd_client = self.create_client(CommandLong, self.command_service)
        if self.wait_for_service:
            if not self._cmd_client.wait_for_service(timeout_sec=self.service_timeout_sec):
                self.get_logger().error(f"MAVROS service '{self.command_service}' unavailable")
                return

        self._mavros_ready = True
        self.get_logger().info(f"MAVROS ready, driving channels {self.pwm_channels}")

  
    def _traj_cb(self, msg: JointTrajectory):
        if not msg.points or not self._mavros_ready:
            return

        point = msg.points[-1]
        name_to_index = {name: i for i, name in enumerate(msg.joint_names)}

        for i, joint in enumerate(self.joint_names):
            if i >= len(self.pwm_channels):
                continue
            js_idx = name_to_index.get(joint)
            if js_idx is None or js_idx >= len(point.positions):
                continue

            angle = float(point.positions[js_idx])
            clamped = clamp(angle, self.angle_min_rad[i], self.angle_max_rad[i])
            pulse = self._angle_to_pwm(i, clamped)

            self._set_servo(self.pwm_channels[i], pulse)
            self._last_positions[joint] = clamped

    def _angle_to_pwm(self, idx: int, angle: float) -> float:
        lower = self.angle_min_rad[idx]
        upper = self.angle_max_rad[idx]
        ratio = (angle - lower) / (upper - lower)
        pwm = self.pulse_min_us + ratio * (self.pulse_max_us - self.pulse_min_us)
        return clamp(pwm, self.pulse_min_us, self.pulse_max_us)

    def _set_servo(self, channel: int, pulse_us: float):
        if not self._mavros_ready or self._cmd_client is None:
            return

        req = CommandLong.Request()
        req.command = MAV_CMD_DO_SET_SERVO
        req.confirmation = 0
        req.param1 = float(channel)
        req.param2 = float(pulse_us)
        req.param3 = 0
        req.param4 = 0
        req.param5 = 0
        req.param6 = 0
        req.param7 = 0

        future = self._cmd_client.call_async(req)
        self._pending_calls.add(future)
        future.add_done_callback(self._done_cb)

    def _done_cb(self, future: Future):
        self._pending_calls.discard(future)
        try:
            resp = future.result()
            if resp and not resp.success:
                self.get_logger().warn(f"PWM command rejected: {resp.result}")
        except Exception as e:
            self.get_logger().error(f"MAVROS call failed: {e}")

    def _write_positions(self, positions: Dict[str, float], log=True):
        for i, joint in enumerate(self.joint_names):
            if i >= len(self.pwm_channels):
                continue
            pulse = self._angle_to_pwm(i, positions[joint])
            self._set_servo(self.pwm_channels[i], pulse)
        if log:
            self.get_logger().info("Initial positions applied")



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
