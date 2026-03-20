from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                output="screen",
                parameters=[{"dev": "/dev/input/js0"}],
            ),
            Node(
                package="arm_teleop",
                executable="joy_to_servo",
                name="joy_to_servo",
                output="screen",
                parameters=[
                    {"linear_scale": 0.15, "angular_scale": 0.55, "deadzone": 0.08, "axis_map": [0, 1, 3, 4]}
                ],
            ),
        ]
    )
