import math
from typing import List

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class LogitechIKToTrajectory(Node):
    def __init__(self) -> None:
        super().__init__("logitech_ik_to_trajectory")

        self.declare_parameter("pose_topic", "ik_target_pose")
        self.declare_parameter("joint3_topic", "joint3_command")
        self.declare_parameter("joint4_topic", "joint4_command")
        self.declare_parameter("joint_state_topic", "joint_states")
        self.declare_parameter("trajectory_topic", "arm_controller/joint_trajectory")

        self.declare_parameter("publish_rate_hz", 15.0)
        self.declare_parameter("time_from_start", 0.12)

        self.declare_parameter("joint_names", ["joint1", "joint2", "joint3", "joint4"])
        self.declare_parameter("lower_limits", [-3.14, -2.5, -2.5, -3.14])
        self.declare_parameter("upper_limits", [3.14, 2.5, 2.5, 3.14])
        self.declare_parameter("initial_positions", [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("joint1_origin_z", 0.10)
        self.declare_parameter("link1_length", 1.27)
        self.declare_parameter("link2_length", 1.27)
        self.declare_parameter("link3_length", 1.27)
        self.declare_parameter("maintain_gripper_level", True)
        self.declare_parameter("gripper_level_joint2_scale", -1.0)

        self.pose_topic = str(self.get_parameter("pose_topic").value)
        self.joint3_topic = str(self.get_parameter("joint3_topic").value)
        self.joint4_topic = str(self.get_parameter("joint4_topic").value)
        self.joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self.trajectory_topic = str(self.get_parameter("trajectory_topic").value)

        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.dt = 1.0 / self.publish_rate_hz
        self.time_from_start = float(self.get_parameter("time_from_start").value)

        self.joint_names: List[str] = list(self.get_parameter("joint_names").value)
        self.lower_limits: List[float] = [float(v) for v in list(self.get_parameter("lower_limits").value)]
        self.upper_limits: List[float] = [float(v) for v in list(self.get_parameter("upper_limits").value)]
        self.current_positions: List[float] = [float(v) for v in list(self.get_parameter("initial_positions").value)]
        self.joint1_origin_z = float(self.get_parameter("joint1_origin_z").value)
        self.link1_length = float(self.get_parameter("link1_length").value)
        self.link2_length = float(self.get_parameter("link2_length").value)
        self.link3_length = float(self.get_parameter("link3_length").value)
        self.maintain_gripper_level = bool(self.get_parameter("maintain_gripper_level").value)
        self.gripper_level_joint2_scale = float(self.get_parameter("gripper_level_joint2_scale").value)

        self.latest_pose: PoseStamped | None = None
        self.latest_joint3_command = 0.0
        self.latest_joint4_command = 0.0
        self.desired_joint3 = self.current_positions[2]
        self.desired_joint4 = self.current_positions[3]
        self.last_joint12 = [self.current_positions[0], self.current_positions[1]]
        self.state_initialized = False
        self.publisher = self.create_publisher(JointTrajectory, self.trajectory_topic, 10)

        self.create_subscription(PoseStamped, self.pose_topic, self._pose_cb, 10)
        self.create_subscription(Float64, self.joint3_topic, self._joint3_cb, 10)
        self.create_subscription(Float64, self.joint4_topic, self._joint4_cb, 10)
        self.create_subscription(JointState, self.joint_state_topic, self._joint_state_cb, 10)
        self.timer = self.create_timer(self.dt, self._tick)

        self.get_logger().info(
            "IK bridge active: left-stick Cartesian target -> analytic joint1/joint2 IK, "
            "right-side controls remain direct on joint3/joint4."
        )

    def _pose_cb(self, msg: PoseStamped) -> None:
        self.latest_pose = msg

    def _joint3_cb(self, msg: Float64) -> None:
        self.latest_joint3_command = float(msg.data)

    def _joint4_cb(self, msg: Float64) -> None:
        self.latest_joint4_command = float(msg.data)

    def _joint_state_cb(self, msg: JointState) -> None:
        index_by_name = {name: idx for idx, name in enumerate(msg.name)}
        updated = False
        for joint_index, joint_name in enumerate(self.joint_names):
            msg_index = index_by_name.get(joint_name)
            if msg_index is None or msg_index >= len(msg.position):
                continue
            self.current_positions[joint_index] = float(msg.position[msg_index])
            updated = True

        if updated and not self.state_initialized:
            self.state_initialized = True

    def _tick(self) -> None:
        if not self.state_initialized or self.latest_pose is None:
            return

        self.desired_joint3 = clamp(
            self.desired_joint3 + (self.latest_joint3_command * self.dt),
            self.lower_limits[2],
            self.upper_limits[2],
        )
        self.desired_joint4 = clamp(
            self.desired_joint4 + (self.latest_joint4_command * self.dt),
            self.lower_limits[3],
            self.upper_limits[3],
        )

        joint1, joint2 = self._solve_joint12(self.latest_pose)
        self.last_joint12 = [joint1, joint2]

        commanded_joint4 = self.desired_joint4
        if self.maintain_gripper_level:
            # Keep the tool approximately level during left-stick vertical motion.
            # The operator's joint4 input becomes an offset around the leveled wrist.
            commanded_joint4 += self.gripper_level_joint2_scale * joint2

        self._publish_target(joint1, joint2, self.desired_joint3, commanded_joint4)

    def _solve_joint12(self, pose: PoseStamped) -> tuple[float, float]:
        # With the original arm convention, joint1 rotates about link1's axis and
        # joint2 rotates about link2's axis. That gives a 2-DOF Cartesian surface
        # for the tool origin. The left stick commands y/z on that surface.
        shoulder_z = self.joint1_origin_z - self.link1_length
        max_joint2 = clamp(self.upper_limits[1], 0.0, math.pi)
        target_z = clamp(
            float(pose.pose.position.z),
            shoulder_z - self.link3_length,
            shoulder_z - (self.link3_length * math.cos(max_joint2)),
        )

        cos_joint2 = clamp((shoulder_z - target_z) / self.link3_length, -1.0, 1.0)
        joint2 = math.acos(cos_joint2)

        radial_y = self.link3_length * math.sin(joint2)
        radius = math.hypot(self.link2_length, radial_y)
        if radius < 1.0e-6:
            joint1 = 0.0
        else:
            target_y = clamp(float(pose.pose.position.y), -radius, radius)
            theta = math.asin(clamp(target_y / radius, -1.0, 1.0))
            phi = math.atan2(radial_y, self.link2_length)
            joint1 = phi - theta

        return (
            clamp(joint1, self.lower_limits[0], self.upper_limits[0]),
            clamp(joint2, self.lower_limits[1], self.upper_limits[1]),
        )

    def _publish_target(self, joint1: float, joint2: float, joint3: float, joint4: float) -> None:
        trajectory = JointTrajectory()
        trajectory.joint_names = list(self.joint_names)

        point = JointTrajectoryPoint()
        point.positions = [
            clamp(joint1, self.lower_limits[0], self.upper_limits[0]),
            clamp(joint2, self.lower_limits[1], self.upper_limits[1]),
            clamp(joint3, self.lower_limits[2], self.upper_limits[2]),
            clamp(joint4, self.lower_limits[3], self.upper_limits[3]),
        ]
        point.time_from_start = Duration(
            sec=int(self.time_from_start),
            nanosec=int((self.time_from_start - int(self.time_from_start)) * 1e9),
        )

        trajectory.points.append(point)
        self.publisher.publish(trajectory)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LogitechIKToTrajectory()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
