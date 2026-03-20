from typing import List

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import Joy


class JoyToServo(Node):
    def __init__(self) -> None:
        super().__init__("joy_to_servo")
        self.declare_parameter("linear_scale", 0.15)
        self.declare_parameter("angular_scale", 0.6)
        self.declare_parameter("deadzone", 0.08)
        self.declare_parameter("axis_map", [0, 1, 3, 4])

        self.linear_scale: float = float(self.get_parameter("linear_scale").value)
        self.angular_scale: float = float(self.get_parameter("angular_scale").value)
        self.deadzone: float = float(self.get_parameter("deadzone").value)
        self.axis_map: List[int] = list(self.get_parameter("axis_map").value)

        self.last_axes: List[float] = [0.0] * 8

        self.sub = self.create_subscription(Joy, "/joy", self.handle_joy, 10)
        self.pub = self.create_publisher(TwistStamped, "/servo_server/delta_twist_cmds", 10)

        self.get_logger().info(
            f"JoyToServo started: linear_scale={self.linear_scale} angular_scale={self.angular_scale} deadzone={self.deadzone}"
        )

    def handle_joy(self, msg: Joy) -> None:
        axes = list(msg.axes)
        if len(axes) < 5:
            return

        mapped = [0.0, 0.0, 0.0, 0.0]
        for i in range(min(4, len(self.axis_map))):
            idx = self.axis_map[i]
            if 0 <= idx < len(axes):
                mapped[i] = axes[idx]

        for i in range(4):
            if abs(mapped[i]) < self.deadzone:
                mapped[i] = 0.0

        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.twist.linear.x = self.linear_scale * mapped[1]
        twist.twist.linear.y = self.linear_scale * mapped[0]
        twist.twist.linear.z = 0.0
        twist.twist.angular.x = 0.0
        twist.twist.angular.y = self.angular_scale * mapped[4] if len(axes) > 4 else 0.0
        twist.twist.angular.z = self.angular_scale * mapped[3]

        self.pub.publish(twist)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JoyToServo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
