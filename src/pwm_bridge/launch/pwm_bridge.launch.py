from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import PushRosNamespace
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    trajectory_topic = LaunchConfiguration("trajectory_topic")
    joint_names = LaunchConfiguration("joint_names")
    rc_channels = LaunchConfiguration("rc_channels")
    rc_override_topic = LaunchConfiguration("rc_override_topic")
    angle_min_rad = LaunchConfiguration("angle_min_rad")
    angle_max_rad = LaunchConfiguration("angle_max_rad")
    pulse_min_us = LaunchConfiguration("pulse_min_us")
    pulse_max_us = LaunchConfiguration("pulse_max_us")
    initial_positions_rad = LaunchConfiguration("initial_positions_rad")

    namespace = ""

    mavros = GroupAction(
        actions=[PushRosNamespace(namespace),
                IncludeLaunchDescription(XMLLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare('arm_pwm_bridge'),
                                          'launch',
                                          'mavros.launch'])
                ), launch_arguments={'fcu_url': 'udp://0.0.0.0:14551@'}.items()
            )
        ]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "trajectory_topic",
                default_value="arm_controller/joint_trajectory"
            ),
            DeclareLaunchArgument(
                "joint_names",
                default_value="['joint1','joint2','joint3','joint4','joint5']"
            ),
            DeclareLaunchArgument(
                "rc_channels",
                default_value="[12,13,14,15,16]",
                description="RC override channels"
            ),
            DeclareLaunchArgument(
                "rc_override_topic",
                default_value="/mavros/rc/override",
                description="RC override output topic"
            ),
            DeclareLaunchArgument(
                "angle_min_rad",
                default_value="[-2.6,-2.0,-2.6,-2.6,-1.5]"
            ),
            DeclareLaunchArgument(
                "angle_max_rad",
                default_value="[2.6,2.6,2.6,2.6,1.5]"
            ),
            DeclareLaunchArgument(
                "pulse_min_us",
                default_value="700.0"
            ),
            DeclareLaunchArgument(
                "pulse_max_us",
                default_value="2300.0"
            ),
            DeclareLaunchArgument(
                "initial_positions_rad",
                default_value="[0.0,0.0,0.0,0.0,0.0]"
            ),
            
            mavros,

            Node(
                package="arm_pwm_bridge",
                executable="joint_traj_to_pwm",
                name="joint_traj_to_pwm",
                output="screen",
                parameters=[
                    {
                        "trajectory_topic": trajectory_topic,
                        "joint_names": joint_names,
                        "rc_channels": rc_channels,
                        "rc_override_topic": rc_override_topic,
                        "angle_min_rad": angle_min_rad,
                        "angle_max_rad": angle_max_rad,
                        "pulse_min_us": ParameterValue(pulse_min_us, value_type=float),
                        "pulse_max_us": ParameterValue(pulse_max_us, value_type=float),
                        "initial_positions_rad": initial_positions_rad,
                    }
                ],
            ),
        ]
    )
