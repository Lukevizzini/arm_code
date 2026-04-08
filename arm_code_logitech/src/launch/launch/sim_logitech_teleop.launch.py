from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    controllers_file = LaunchConfiguration("controllers_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_rviz = LaunchConfiguration("start_rviz")
    automatic_discovery_range = LaunchConfiguration("automatic_discovery_range")
    device_id = LaunchConfiguration("device_id")
    joy_deadzone = LaunchConfiguration("joy_deadzone")
    autorepeat_rate = LaunchConfiguration("autorepeat_rate")
    publish_rate_hz = LaunchConfiguration("publish_rate_hz")
    mapper_deadzone = LaunchConfiguration("mapper_deadzone")
    position_velocity_mps = LaunchConfiguration("position_velocity_mps")
    joint3_velocity_radps = LaunchConfiguration("joint3_velocity_radps")
    joint4_velocity_radps = LaunchConfiguration("joint4_velocity_radps")
    joint4_right_button = LaunchConfiguration("joint4_right_button")
    joint4_left_button = LaunchConfiguration("joint4_left_button")
    fixed_x = LaunchConfiguration("fixed_x")
    initial_y = LaunchConfiguration("initial_y")
    initial_z = LaunchConfiguration("initial_z")
    left_stick_x_scale = LaunchConfiguration("left_stick_x_scale")
    left_stick_y_scale = LaunchConfiguration("left_stick_y_scale")
    right_stick_joint3_scale = LaunchConfiguration("right_stick_joint3_scale")
    gripper_scale = LaunchConfiguration("gripper_scale")
    trigger_released_value = LaunchConfiguration("trigger_released_value")
    trigger_pressed_value = LaunchConfiguration("trigger_pressed_value")
    ik_publish_rate_hz = LaunchConfiguration("ik_publish_rate_hz")
    ik_time_from_start = LaunchConfiguration("ik_time_from_start")

    controllers_default = PathJoinSubstitution(
        [FindPackageShare("arm_config"), "config", "ros2_controllers.yaml"]
    )

    ros2_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("arm_bringup"), "launch", "ros2_control.launch.py"])
        ),
        launch_arguments={
            "controllers_file": controllers_file,
            "use_sim_time": use_sim_time,
            "start_robot_state_publisher": "false",
        }.items(),
    )

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("arm_launch"), "launch", "move_group.launch.py"])
        ),
        launch_arguments={
            "controllers_file": controllers_file,
            "use_sim_time": use_sim_time,
            "start_rviz": start_rviz,
        }.items(),
    )

    logitech_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("arm_launch"), "launch", "logitech_ik_teleop.launch.py"])
        ),
        launch_arguments={
            "device_id": device_id,
            "joy_deadzone": joy_deadzone,
            "autorepeat_rate": autorepeat_rate,
            "publish_rate_hz": publish_rate_hz,
            "mapper_deadzone": mapper_deadzone,
            "position_velocity_mps": position_velocity_mps,
            "joint3_velocity_radps": joint3_velocity_radps,
            "joint4_velocity_radps": joint4_velocity_radps,
            "joint4_right_button": joint4_right_button,
            "joint4_left_button": joint4_left_button,
            "fixed_x": fixed_x,
            "initial_y": initial_y,
            "initial_z": initial_z,
            "left_stick_x_scale": left_stick_x_scale,
            "left_stick_y_scale": left_stick_y_scale,
            "right_stick_joint3_scale": right_stick_joint3_scale,
            "gripper_scale": gripper_scale,
            "trigger_released_value": trigger_released_value,
            "trigger_pressed_value": trigger_pressed_value,
        }.items(),
    )

    sim_ik_node = Node(
        package="arm_teleop",
        executable="logitech_ik_to_trajectory",
        name="logitech_ik_to_trajectory",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "publish_rate_hz": ParameterValue(ik_publish_rate_hz, value_type=float),
                "time_from_start": ParameterValue(ik_time_from_start, value_type=float),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "controllers_file",
                default_value=controllers_default,
                description="Controller config shared by ros2_control and MoveIt",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            DeclareLaunchArgument(
                "automatic_discovery_range",
                default_value="LOCALHOST",
                description="Keep the sim local-only by default. Use SUBNET to allow remote ROS nodes.",
            ),
            DeclareLaunchArgument("device_id", default_value="0"),
            DeclareLaunchArgument("joy_deadzone", default_value="0.05"),
            DeclareLaunchArgument("autorepeat_rate", default_value="50.0"),
            DeclareLaunchArgument("publish_rate_hz", default_value="30.0"),
            DeclareLaunchArgument("mapper_deadzone", default_value="0.12"),
            DeclareLaunchArgument("position_velocity_mps", default_value="0.30"),
            DeclareLaunchArgument("joint3_velocity_radps", default_value="1.20"),
            DeclareLaunchArgument("joint4_velocity_radps", default_value="1.20"),
            DeclareLaunchArgument("joint4_right_button", default_value="5"),
            DeclareLaunchArgument("joint4_left_button", default_value="4"),
            DeclareLaunchArgument("fixed_x", default_value="1.27"),
            DeclareLaunchArgument("initial_y", default_value="0.0"),
            DeclareLaunchArgument("initial_z", default_value="-2.44"),
            DeclareLaunchArgument("left_stick_x_scale", default_value="1.0"),
            DeclareLaunchArgument("left_stick_y_scale", default_value="1.0"),
            DeclareLaunchArgument("right_stick_joint3_scale", default_value="-1.0"),
            DeclareLaunchArgument("gripper_scale", default_value="-1.0"),
            DeclareLaunchArgument("trigger_released_value", default_value="1.0"),
            DeclareLaunchArgument("trigger_pressed_value", default_value="-1.0"),
            DeclareLaunchArgument("ik_publish_rate_hz", default_value="15.0"),
            DeclareLaunchArgument("ik_time_from_start", default_value="0.12"),
            SetEnvironmentVariable("ROS_AUTOMATIC_DISCOVERY_RANGE", automatic_discovery_range),
            ros2_control_launch,
            move_group_launch,
            logitech_launch,
            sim_ik_node,
        ]
    )
