import sys
import termios
import tty

from arm_teleop.keyboard_to_joy import KeyboardToJoy
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

KEY_MAPPING = {
    "w": (0.01, 0, 0, 0, 0, 0),
    "s": (-0.01, 0, 0, 0, 0, 0),
    "a": (0, 0.01, 0, 0, 0, 0),
    "d": (0, -0.01, 0, 0, 0, 0),
    "i": (0, 0, 0.01, 0, 0, 0),
    "k": (0, 0, -0.01, 0, 0, 0),
    "j": (0, 0, 0, 0, 0, 0.05),
    "l": (0, 0, 0, 0, 0, -0.05),
}


def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key


class KeyboardServo(Node):
    def __init__(self):
        super().__init__("keyboard_servo")
        self.pub = self.create_publisher(Twist, "/servo_server/delta_twist_cmds", 10)
        self.timer = self.create_timer(0.05, self.publish_twist)

    def publish_twist(self):
        try:
            key = get_key()
            if key == "\x03":  # Ctrl+C
                raise KeyboardInterrupt
            if key in KEY_MAPPING:
                lx, ly, lz, ax, ay, az = KEY_MAPPING[key]
                twist = Twist()
                twist.linear.x = lx
                twist.linear.y = ly
                twist.linear.z = lz
                twist.angular.x = ax
                twist.angular.y = ay
                twist.angular.z = az
                self.pub.publish(twist)
        except KeyboardInterrupt:
            raise
        except Exception as e:
            self.get_logger().error(str(e))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KeyboardServo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
