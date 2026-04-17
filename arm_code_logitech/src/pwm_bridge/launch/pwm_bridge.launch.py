from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    trajectory_topic = LaunchConfiguration("trajectory_topic")
    joint_names = LaunchConfiguration("joint_names")
    pwm_channels = LaunchConfiguration("pwm_channels")
    rc_override_topic = LaunchConfiguration("rc_override_topic")
    gripper_topic = LaunchConfiguration("gripper_topic")
    gripper_channel = LaunchConfiguration("gripper_channel")
    angle_min_rad = LaunchConfiguration("angle_min_rad")
    angle_max_rad = LaunchConfiguration("angle_max_rad")
    pulse_min_us = LaunchConfiguration("pulse_min_us")
    pulse_max_us = LaunchConfiguration("pulse_max_us")
    pulse_min_us_per_joint = LaunchConfiguration("pulse_min_us_per_joint")
    pulse_max_us_per_joint = LaunchConfiguration("pulse_max_us_per_joint")
    gripper_command_min = LaunchConfiguration("gripper_command_min")
    gripper_command_max = LaunchConfiguration("gripper_command_max")
    gripper_pulse_min_us = LaunchConfiguration("gripper_pulse_min_us")
    gripper_pulse_max_us = LaunchConfiguration("gripper_pulse_max_us")
    gripper_command_is_rate = LaunchConfiguration("gripper_command_is_rate")
    gripper_rate_pwm_per_sec = LaunchConfiguration("gripper_rate_pwm_per_sec")
    initial_gripper_pwm_us = LaunchConfiguration("initial_gripper_pwm_us")
    initial_positions_rad = LaunchConfiguration("initial_positions_rad")
    initial_gripper_command = LaunchConfiguration("initial_gripper_command")
    namespace = ""

    mavros = GroupAction(
        actions=[
            PushRosNamespace(namespace),
            IncludeLaunchDescription(
                XMLLaunchDescriptionSource(
                    [FindPackageShare("arm_pwm_bridge"), "/launch/mavros.launch"]
                ),
                launch_arguments={"fcu_url": "udp://0.0.0.0:14551@"}.items(),
            ),
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("trajectory_topic", default_value="arm_controller/joint_trajectory"),
            DeclareLaunchArgument("joint_names", default_value="['joint1','joint2','joint3','joint4']"),
            DeclareLaunchArgument("pwm_channels", default_value="[12,13,14,15]", description="RC override channels"),
            DeclareLaunchArgument("rc_override_topic", default_value="/mavros/rc/override"),
            DeclareLaunchArgument("gripper_topic", default_value="/gripper_open_close_command"),
            DeclareLaunchArgument(
                "gripper_channel",
                default_value="16",
                description="RC override channel for the gripper. Set <=0 to disable.",
            ),
            DeclareLaunchArgument("angle_min_rad", default_value="[-2.6,-2.6,-2.6,-2.6]"),
            DeclareLaunchArgument("angle_max_rad", default_value="[2.6,2.6,2.6,2.6]"),
            DeclareLaunchArgument("pulse_min_us", default_value="700.0"),
            DeclareLaunchArgument("pulse_max_us", default_value="2300.0"),
            DeclareLaunchArgument("gripper_command_min", default_value="-1.0"),
            DeclareLaunchArgument("gripper_command_max", default_value="1.0"),
            DeclareLaunchArgument("gripper_pulse_min_us", default_value="900.0"),
            DeclareLaunchArgument("gripper_pulse_max_us", default_value="1500.0"),
            DeclareLaunchArgument(
                "gripper_command_is_rate",
                default_value="true",
                description="Interpret gripper input as rate; zero input holds current PWM.",
            ),
            DeclareLaunchArgument("gripper_rate_pwm_per_sec", default_value="400.0"),
            DeclareLaunchArgument("initial_gripper_pwm_us", default_value="1500.0"),
            DeclareLaunchArgument("initial_positions_rad", default_value="[0.0,0.0,0.0,0.0]"),
            mavros,
            Node(
                package="arm_pwm_bridge",
                executable="joint_traj_to_pwm",
                output="screen",
                parameters=[
                    {
                        "trajectory_topic": trajectory_topic,
                        "joint_names": ParameterValue(joint_names, value_type=str),
                        "rc_channels": ParameterValue(pwm_channels, value_type=str),
                        "rc_override_topic": rc_override_topic,
                        "gripper_topic": gripper_topic,
                        "gripper_channel": ParameterValue(gripper_channel, value_type=int),
                        "angle_min_rad": ParameterValue(angle_min_rad, value_type=str),
                        "angle_max_rad": ParameterValue(angle_max_rad, value_type=str),
                        "pulse_min_us": ParameterValue(pulse_min_us, value_type=float),
                        "pulse_max_us": ParameterValue(pulse_max_us, value_type=float),
                        "gripper_command_min": ParameterValue(gripper_command_min, value_type=float),
                        "gripper_command_max": ParameterValue(gripper_command_max, value_type=float),
                        "gripper_pulse_min_us": ParameterValue(gripper_pulse_min_us, value_type=float),
                        "gripper_pulse_max_us": ParameterValue(gripper_pulse_max_us, value_type=float),
                        "gripper_command_is_rate": ParameterValue(gripper_command_is_rate, value_type=bool),
                        "gripper_rate_pwm_per_sec": ParameterValue(gripper_rate_pwm_per_sec, value_type=float),
                        "initial_gripper_pwm_us": ParameterValue(initial_gripper_pwm_us, value_type=float),
                        "initial_positions_rad": ParameterValue(initial_positions_rad, value_type=str),
                    }
                ],
            ),
        ]
    )
