import sys
import termios
import threading
import time
import tty

import rclpy
from control_msgs.msg import JointJog
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

TRANSLATION_BINDINGS = {
    "w": (0.18, 0.0, 0.0),
    "s": (-0.18, 0.0, 0.0),
    "a": (0.0, 0.18, 0.0),
    "d": (0.0, -0.18, 0.0),
    "i": (0.0, 0.0, 0.18),
    "k": (0.0, 0.0, -0.18),
}

ROLL_BINDINGS = {
    "j": 0.12,
    "l": -0.12,
}

YAW2_BINDINGS = {
    "u": 0.10,
    "o": -0.10,
}

SINGULARITY_ESCAPE = {
    "joint2": 0.16,
    "joint3": -0.16,
}

JOINT_LIMITS = {
    "joint1": (-3.14, 3.14),
    "joint2": (-2.5, 2.5),
    "joint3": (-2.5, 2.5),
    "joint4": (-3.14, 3.14),
}

# Keep translation commands out of the near-singularity zone.
SINGULARITY_GUARD_RAD = 0.12

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
        self.twist_pub = self.create_publisher(TwistStamped, "/servo_server/delta_twist_cmds", 10)
        self.joint_pub = self.create_publisher(JointJog, "/servo_server/delta_joint_cmds", 10)
        self.traj_pub = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)
        self.start_servo_client = self.create_client(Trigger, "/servo_server/start_servo")
        self.joint_positions = {"joint1": 0.0, "joint2": 0.0, "joint3": 0.0, "joint4": 0.0}
        self.have_joint_state = False
        self.last_escape_time = 0.0
        self.escape_cooldown_sec = 1.5
        self.create_subscription(JointState, "/joint_states", self.handle_joint_state, 10)
        self.thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self.thread.start()

    def ensure_servo_started(self) -> None:
        if not self.start_servo_client.wait_for_service(timeout_sec=0.2):
            return
        future = self.start_servo_client.call_async(Trigger.Request())

        def _done(fut):
            try:
                result = fut.result()
                if result is not None and not result.success:
                    self.get_logger().warn(f"start_servo returned: {result.message}")
            except Exception as exc:
                self.get_logger().warn(f"start_servo call failed: {exc}")

        future.add_done_callback(_done)

    def handle_joint_state(self, msg: JointState) -> None:
        for joint_name in self.joint_positions:
            if joint_name in msg.name:
                self.joint_positions[joint_name] = msg.position[msg.name.index(joint_name)]
                self.have_joint_state = True

    def is_straight_arm_singularity(self) -> bool:
        if not self.have_joint_state:
            return False
        return (
            abs(self.joint_positions["joint2"]) < 0.04
            and abs(self.joint_positions["joint3"]) < 0.04
        )

    def is_near_singularity(self) -> bool:
        if not self.have_joint_state:
            return False
        return (
            abs(self.joint_positions["joint2"]) < SINGULARITY_GUARD_RAD
            and abs(self.joint_positions["joint3"]) < SINGULARITY_GUARD_RAD
        )

    def escape_singularity(self) -> None:
        now = time.monotonic()
        if now - self.last_escape_time < self.escape_cooldown_sec:
            return
        self.last_escape_time = now

        # Publish directly to the controller so we can always bend out of the
        # straight-arm singularity, even when Servo is emergency-stopped.
        target = {
            "joint1": self.joint_positions["joint1"],
            "joint2": self.joint_positions["joint2"] + SINGULARITY_ESCAPE["joint2"],
            "joint3": self.joint_positions["joint3"] + SINGULARITY_ESCAPE["joint3"],
            "joint4": self.joint_positions["joint4"],
        }
        for name, value in target.items():
            lower, upper = JOINT_LIMITS[name]
            target[name] = max(lower, min(upper, value))

        # Publish a short burst to increase reliability when controller timing is tight.
        for _ in range(6):
            traj = JointTrajectory()
            traj.joint_names = ["joint1", "joint2", "joint3", "joint4"]
            pt = JointTrajectoryPoint()
            pt.positions = [
                target["joint1"],
                target["joint2"],
                target["joint3"],
                target["joint4"],
            ]
            pt.time_from_start.sec = 0
            pt.time_from_start.nanosec = 350_000_000
            traj.points = [pt]
            self.traj_pub.publish(traj)
            time.sleep(0.03)

        self.ensure_servo_started()

        traj_subs = self.traj_pub.get_subscription_count()
        self.get_logger().warn(
            f"Arm is at the straight-arm singularity. Sent local nudge burst (trajectory subscribers: {traj_subs}); press your Cartesian key again."
        )

    def publish_translation(self, x: float, y: float, z: float) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = x
        msg.twist.linear.y = y
        msg.twist.linear.z = z
        self.twist_pub.publish(msg)

    def publish_roll(self, delta: float) -> None:
        self.publish_joint_delta("joint4", delta, 250_000_000)

    def publish_yaw2(self, delta: float) -> None:
        self.publish_joint_delta("joint3", delta, 250_000_000)

    def publish_joint_delta(self, joint_name: str, delta: float, duration_ns: int) -> None:
        if self.have_joint_state:
            lower, upper = JOINT_LIMITS[joint_name]
            target_joint = max(lower, min(upper, self.joint_positions[joint_name] + delta))

            traj = JointTrajectory()
            traj.joint_names = ["joint1", "joint2", "joint3", "joint4"]
            pt = JointTrajectoryPoint()
            positions = {
                "joint1": self.joint_positions["joint1"],
                "joint2": self.joint_positions["joint2"],
                "joint3": self.joint_positions["joint3"],
                "joint4": self.joint_positions["joint4"],
            }
            positions[joint_name] = target_joint
            pt.positions = [
                positions["joint1"],
                positions["joint2"],
                positions["joint3"],
                positions["joint4"],
            ]
            pt.time_from_start.sec = 0
            pt.time_from_start.nanosec = duration_ns
            traj.points = [pt]
            self.traj_pub.publish(traj)

    def keyboard_loop(self):
        self.ensure_servo_started()
        self.get_logger().info(
            "Keyboard ready. W/S X, A/D Y, I/K Z, U/O yaw2 (joint3), J/L gripper roll (joint4), H bend-out-of-singularity. Ctrl+C to quit."
        )
        while rclpy.ok():
            try:
                key = get_key()
                if key == "\x03":  # Ctrl+C
                    break
                if key in TRANSLATION_BINDINGS:
                    if self.is_near_singularity():
                        self.escape_singularity()
                        self.get_logger().warn(
                            "Translation blocked near singularity guard. Press command again after the recovery nudge."
                        )
                        continue
                    x, y, z = TRANSLATION_BINDINGS[key]
                    self.publish_translation(x, y, z)
                elif key in ROLL_BINDINGS:
                    self.publish_roll(ROLL_BINDINGS[key])
                elif key in YAW2_BINDINGS:
                    self.publish_yaw2(YAW2_BINDINGS[key])
                elif key == "h":
                    self.escape_singularity()
            except Exception as e:
                self.get_logger().error(str(e))


def main(args=None):
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