from typing import List

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

from arm_teleop.logitech_mapping import LogitechInterpreter, LogitechMappingConfig


def _read_limits(node: Node, parameter_name: str, default: List[float]) -> tuple[float, float]:
    limits = [float(value) for value in list(node.get_parameter(parameter_name).value)]
    if len(limits) != 2:
        node.get_logger().warn(
            f"Parameter '{parameter_name}' must contain exactly two values. Falling back to {default}."
        )
        return (default[0], default[1])
    lower = min(limits[0], limits[1])
    upper = max(limits[0], limits[1])
    return (lower, upper)


class LogitechJoyToIK(Node):
    def __init__(self) -> None:
        super().__init__("logitech_joy_to_ik")

        self.declare_parameter("joy_topic", "joy")
        self.declare_parameter("pose_topic", "ik_target_pose")
        self.declare_parameter("joint3_topic", "joint3_command")
        self.declare_parameter("joint4_topic", "joint4_command")
        self.declare_parameter("gripper_topic", "gripper_open_close_command")
        self.declare_parameter("base_frame", "base_link")

        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("deadzone", 0.12)
        self.declare_parameter("position_velocity_mps", 0.30)
        self.declare_parameter("joint3_velocity_radps", 1.20)
        self.declare_parameter("joint4_velocity_radps", 1.20)

        self.declare_parameter("fixed_x", 1.27)
        self.declare_parameter("initial_y", 0.0)
        self.declare_parameter("initial_z", -2.44)
        self.declare_parameter("y_limits", [-1.50, 1.50])
        self.declare_parameter("z_limits", [-3.20, -0.20])

        self.declare_parameter("left_stick_x_axis", 0)
        self.declare_parameter("left_stick_y_axis", 1)
        self.declare_parameter("right_stick_joint3_axis", 3)
        self.declare_parameter("joint4_right_button", 5)
        self.declare_parameter("joint4_left_button", 4)
        self.declare_parameter("gripper_open_axis", 2)
        self.declare_parameter("gripper_close_axis", 5)
        self.declare_parameter("left_stick_x_scale", 1.0)
        self.declare_parameter("left_stick_y_scale", 1.0)
        self.declare_parameter("right_stick_joint3_scale", -1.0)
        self.declare_parameter("gripper_scale", -1.0)
        self.declare_parameter("trigger_released_value", 1.0)
        self.declare_parameter("trigger_pressed_value", -1.0)

        joy_topic = str(self.get_parameter("joy_topic").value)
        pose_topic = str(self.get_parameter("pose_topic").value)
        joint3_topic = str(self.get_parameter("joint3_topic").value)
        joint4_topic = str(self.get_parameter("joint4_topic").value)
        gripper_topic = str(self.get_parameter("gripper_topic").value)
        self.base_frame = str(self.get_parameter("base_frame").value)

        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.dt = 1.0 / publish_rate_hz

        config = LogitechMappingConfig(
            deadzone=float(self.get_parameter("deadzone").value),
            position_velocity_mps=float(self.get_parameter("position_velocity_mps").value),
            joint3_velocity_radps=float(self.get_parameter("joint3_velocity_radps").value),
            joint4_velocity_radps=float(self.get_parameter("joint4_velocity_radps").value),
            fixed_x=float(self.get_parameter("fixed_x").value),
            initial_y=float(self.get_parameter("initial_y").value),
            initial_z=float(self.get_parameter("initial_z").value),
            y_limits=_read_limits(self, "y_limits", [-1.50, 1.50]),
            z_limits=_read_limits(self, "z_limits", [-3.20, -0.20]),
            left_stick_x_axis=int(self.get_parameter("left_stick_x_axis").value),
            left_stick_y_axis=int(self.get_parameter("left_stick_y_axis").value),
            right_stick_joint3_axis=int(self.get_parameter("right_stick_joint3_axis").value),
            joint4_right_button=int(self.get_parameter("joint4_right_button").value),
            joint4_left_button=int(self.get_parameter("joint4_left_button").value),
            gripper_open_axis=int(self.get_parameter("gripper_open_axis").value),
            gripper_close_axis=int(self.get_parameter("gripper_close_axis").value),
            left_stick_x_scale=float(self.get_parameter("left_stick_x_scale").value),
            left_stick_y_scale=float(self.get_parameter("left_stick_y_scale").value),
            right_stick_joint3_scale=float(self.get_parameter("right_stick_joint3_scale").value),
            gripper_scale=float(self.get_parameter("gripper_scale").value),
            trigger_released_value=float(self.get_parameter("trigger_released_value").value),
            trigger_pressed_value=float(self.get_parameter("trigger_pressed_value").value),
        )
        self.interpreter = LogitechInterpreter(config)

        self.latest_axes: List[float] = []
        self.latest_buttons: List[int] = []

        self.pose_publisher = self.create_publisher(PoseStamped, pose_topic, 10)
        self.joint3_publisher = self.create_publisher(Float64, joint3_topic, 10)
        self.joint4_publisher = self.create_publisher(Float64, joint4_topic, 10)
        self.gripper_publisher = self.create_publisher(Float64, gripper_topic, 10)

        self.create_subscription(Joy, joy_topic, self._joy_cb, 10)
        self.timer = self.create_timer(self.dt, self._publish)

        self.get_logger().info(
            "Logitech joystick teleop active: left stick -> IK y/z target, "
            f"right stick axis {config.right_stick_joint3_axis} -> joint3 command, "
            f"RB/LB buttons {config.joint4_right_button}/{config.joint4_left_button} -> joint4, "
            f"trigger axes {config.gripper_open_axis}/{config.gripper_close_axis} -> gripper."
        )

    def _joy_cb(self, msg: Joy) -> None:
        self.latest_axes = list(msg.axes)
        self.latest_buttons = list(msg.buttons)

    def _publish(self) -> None:
        output = self.interpreter.step(self.latest_axes, self.latest_buttons, self.dt)

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.base_frame
        pose.pose.position.x = output.pose_x
        pose.pose.position.y = output.pose_y
        pose.pose.position.z = output.pose_z
        pose.pose.orientation.w = 1.0
        self.pose_publisher.publish(pose)

        joint3_msg = Float64()
        joint3_msg.data = output.joint3_command
        self.joint3_publisher.publish(joint3_msg)

        joint4_msg = Float64()
        joint4_msg.data = output.joint4_command
        self.joint4_publisher.publish(joint4_msg)

        gripper_msg = Float64()
        gripper_msg.data = output.gripper_command
        self.gripper_publisher.publish(gripper_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LogitechJoyToIK()
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
