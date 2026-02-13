import math
from typing import Dict, List, Set

import rclpy
from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

try:
    from mavros_msgs.srv import CommandLong  # type: ignore
except ImportError:  # pragma: no cover - runtime dependency, not present in CI by default
    CommandLong = None


MAV_CMD_DO_SET_SERVO = 183


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class JointTrajectoryToPWM(Node):
    """
    Minimal bridge: listens to JointTrajectory and drives Navigator PWM channels
    via MAVROS CommandLong (MAV_CMD_DO_SET_SERVO) service calls.
    """

    def __init__(self) -> None:
        super().__init__("joint_traj_to_pwm")

        self.declare_parameter("trajectory_topic", "arm_controller/joint_trajectory")
        self.declare_parameter("joint_names", ["joint1", "joint2", "joint3", "joint4"])
        self.declare_parameter("pwm_channels", [9, 10, 11, 12])
        self.declare_parameter("pulse_min_us", 700.0)
        self.declare_parameter("pulse_max_us", 2300.0)
        self.declare_parameter("angle_min_rad", [-2.6, -2.0, -2.6, -2.6])
        self.declare_parameter("angle_max_rad", [2.6, 2.6, 2.6, 2.6])
        self.declare_parameter("initial_positions_rad", [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("command_service", "/mavros/cmd/command")
        self.declare_parameter("wait_for_service", True)
        self.declare_parameter("service_timeout_sec", 5.0)
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
        self.command_service: str = str(self.get_parameter("command_service").value).strip()
        self.wait_for_service: bool = bool(self.get_parameter("wait_for_service").value)
        self.service_timeout_sec: float = float(self.get_parameter("service_timeout_sec").value)
        self.publish_joint_states: bool = bool(self.get_parameter("publish_joint_states").value)
        self.joint_state_rate_hz: float = float(self.get_parameter("joint_state_rate_hz").value)

        if len(self.joint_names) != len(self.pwm_channels):
            self.get_logger().warn(
                f"joint_names ({len(self.joint_names)}) and pwm_channels ({len(self.pwm_channels)}) length mismatch; "
                "extra joints or channels will be ignored."
            )

        self._cmd_client = None
        self._mavros_ready = False
        self._warned_disabled = False
        self._warned_service_unavailable = False
        self._pending_calls: Set[Future] = set()
        self._last_positions: Dict[str, float] = {
            name: self._value_for_idx(self.initial_positions_rad, idx, 0.0)
            for idx, name in enumerate(self.joint_names)
        }

        self._connect_mavros()

        self.create_subscription(JointTrajectory, self.trajectory_topic, self._traj_cb, 10)
        self.get_logger().info(
            f"Listening on {self.trajectory_topic} for JointTrajectory commands -> MAVROS command service."
        )

        if self.publish_joint_states:
            self._js_pub = self.create_publisher(JointState, "joint_states", 10)
            self.create_timer(1.0 / self.joint_state_rate_hz, self._publish_joint_states)
        else:
            self._js_pub = None

        # Apply initial positions if specified.
        if self._mavros_ready and self._cmd_client and self._cmd_client.service_is_ready():
            self._write_positions(self._last_positions, log=False)

    def _value_for_idx(self, values: List[float], idx: int, default: float) -> float:
        return float(values[idx]) if idx < len(values) else default

    def _connect_mavros(self) -> None:
        if CommandLong is None:
            self.get_logger().error(
                "mavros_msgs is not available. Install ROS mavros messages package to enable MAVROS PWM output."
            )
            return

        if not self.command_service:
            self.get_logger().error(
                "command_service is empty. Set it to a valid service, e.g. /mavros/cmd/command."
            )
            return

        self._cmd_client = self.create_client(CommandLong, self.command_service)  # type: ignore[arg-type]
        if self.wait_for_service and not self._cmd_client.wait_for_service(timeout_sec=self.service_timeout_sec):
            self.get_logger().error(
                f"MAVROS command service '{self.command_service}' unavailable after {self.service_timeout_sec}s."
            )
            return

        for channel in self.pwm_channels:
            if channel <= 0:
                self.get_logger().warn(
                    f"Invalid pwm channel {channel}; channels should be positive (Navigator: 1-16)."
                )

        self._mavros_ready = True
        self.get_logger().info(
            f"MAVROS command client ready on {self.command_service}; driving channels {self.pwm_channels}."
        )

    def _traj_cb(self, msg: JointTrajectory) -> None:
        if not msg.points:
            return
        if not self._mavros_ready or self._cmd_client is None:
            if not self._warned_disabled:
                self.get_logger().warn("PWM disabled (MAVROS command client not ready); dropping messages.")
                self._warned_disabled = True
            return
        if not self._cmd_client.service_is_ready():
            if not self._warned_service_unavailable:
                self.get_logger().warn(
                    f"MAVROS command service '{self.command_service}' is not available yet; dropping messages."
                )
                self._warned_service_unavailable = True
            return
        self._warned_service_unavailable = False

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
            pulse_us = self._angle_to_pulse(idx, clamped)
            self._set_servo(channel, pulse_us)
            updates[joint] = clamped

        if updates:
            self._last_positions.update(updates)

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

    def _set_servo(self, channel: int, pulse_us: float) -> None:
        if not self._mavros_ready or self._cmd_client is None:
            return
        if channel <= 0:
            return
        if not self._cmd_client.service_is_ready():
            return
        request = CommandLong.Request()
        request.broadcast = False
        request.command = MAV_CMD_DO_SET_SERVO
        request.confirmation = 0
        request.param1 = float(channel)
        request.param2 = float(pulse_us)
        request.param3 = 0.0
        request.param4 = 0.0
        request.param5 = 0.0
        request.param6 = 0.0
        request.param7 = 0.0
        try:
            future = self._cmd_client.call_async(request)
            self._pending_calls.add(future)
            future.add_done_callback(self._command_done_cb)
        except Exception as exc:
            self.get_logger().error(f"Failed sending MAVROS DO_SET_SERVO for channel {channel}: {exc}")

    def _command_done_cb(self, future: Future) -> None:
        self._pending_calls.discard(future)
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f"MAVROS command call failed: {exc}")
            return
        if response and not response.success:
            self.get_logger().warn(
                f"MAVROS DO_SET_SERVO rejected (result={response.result}). "
                "Check SERVOx_FUNCTION and vehicle arm/state settings."
            )

    def _write_positions(self, positions: Dict[str, float], log: bool = True) -> None:
        if not self._mavros_ready:
            return
        for idx, joint in enumerate(self.joint_names):
            if idx >= len(self.pwm_channels):
                continue
            channel = self.pwm_channels[idx]
            angle = positions.get(joint, 0.0)
            pulse_us = self._angle_to_pulse(idx, self._clamp_angle(idx, angle))
            self._set_servo(channel, pulse_us)
        if log:
            self.get_logger().info(f"Applied initial positions (us in [{self.pulse_min_us}, {self.pulse_max_us}]).")

    def destroy_node(self) -> bool:
        for future in list(self._pending_calls):
            if not future.done():
                future.cancel()
        self._pending_calls.clear()
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
