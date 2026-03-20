from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    controllers_file = LaunchConfiguration("controllers_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_rviz = LaunchConfiguration("start_rviz")

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("arm_bringup"), "launch", "ros2_control.launch.py"])
        ),
        launch_arguments={
            "controllers_file": controllers_file,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("arm_launch"), "launch", "move_group.launch.py"])
        ),
        launch_arguments={
            "controllers_file": controllers_file,
            "use_sim_time": use_sim_time,
            "start_robot_state_publisher": "false",
            "start_rviz": start_rviz,
        }.items(),
    )

    controllers_default = PathJoinSubstitution([FindPackageShare("arm_config"), "config", "ros2_controllers.yaml"])

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "controllers_file",
                default_value=controllers_default,
                description="ros2_control controller configuration",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false", description="Use simulation clock"),
            DeclareLaunchArgument("start_rviz", default_value="true", description="Start RViz2"),
            LogInfo(msg="Backend started. Run keyboard teleop separately in a real TTY:"),
            LogInfo(msg="  ros2 run arm_teleop keyboard_servo"),
            bringup_launch,
            moveit_launch,
        ]
    )
