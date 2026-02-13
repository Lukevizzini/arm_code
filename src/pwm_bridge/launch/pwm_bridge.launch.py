from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    joint_names = LaunchConfiguration("joint_names")
    pwm_channels = LaunchConfiguration("pwm_channels")
    angle_min_rad = LaunchConfiguration("angle_min_rad")
    angle_max_rad = LaunchConfiguration("angle_max_rad")
    pulse_min_us = LaunchConfiguration("pulse_min_us")
    pulse_max_us = LaunchConfiguration("pulse_max_us")
    command_service = LaunchConfiguration("command_service")
    wait_for_service = LaunchConfiguration("wait_for_service")
    service_timeout_sec = LaunchConfiguration("service_timeout_sec")
    publish_joint_states = LaunchConfiguration("publish_joint_states")

    return LaunchDescription(
        [
            DeclareLaunchArgument("joint_names", default_value="['joint1','joint2','joint3','joint4']"),
            DeclareLaunchArgument("pwm_channels", default_value="[9,10,11,12]", description="Navigator PWM channels"),
            DeclareLaunchArgument("angle_min_rad", default_value="[-2.6,-2.0,-2.6,-2.6]"),
            DeclareLaunchArgument("angle_max_rad", default_value="[2.6,2.6,2.6,2.6]"),
            DeclareLaunchArgument("pulse_min_us", default_value="700.0"),
            DeclareLaunchArgument("pulse_max_us", default_value="2300.0"),
            DeclareLaunchArgument(
                "command_service",
                default_value="/mavros/cmd/command",
                description="MAVROS CommandLong service used for DO_SET_SERVO",
            ),
            DeclareLaunchArgument("wait_for_service", default_value="true"),
            DeclareLaunchArgument("service_timeout_sec", default_value="5.0"),
            DeclareLaunchArgument("publish_joint_states", default_value="true"),
            Node(
                package="arm_pwm_bridge",
                executable="joint_traj_to_pwm",
                output="screen",
                parameters=[
                    {
                        "joint_names": joint_names,
                        "pwm_channels": pwm_channels,
                        "angle_min_rad": angle_min_rad,
                        "angle_max_rad": angle_max_rad,
                        "pulse_min_us": ParameterValue(pulse_min_us, value_type=float),
                        "pulse_max_us": ParameterValue(pulse_max_us, value_type=float),
                        "command_service": command_service,
                        "wait_for_service": ParameterValue(wait_for_service, value_type=bool),
                        "service_timeout_sec": ParameterValue(service_timeout_sec, value_type=float),
                        "publish_joint_states": ParameterValue(publish_joint_states, value_type=bool),
                    }
                ],
            ),
        ]
    )
