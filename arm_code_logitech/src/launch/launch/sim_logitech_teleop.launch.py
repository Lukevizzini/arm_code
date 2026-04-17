from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    controllers_file = LaunchConfiguration("controllers_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_teleop_rviz = LaunchConfiguration("start_teleop_rviz")
    start_move_group = LaunchConfiguration("start_move_group")
    rviz_config = LaunchConfiguration("rviz_config")
    automatic_discovery_range = LaunchConfiguration("automatic_discovery_range")
    device_id = LaunchConfiguration("device_id")
    use_event_device = LaunchConfiguration("use_event_device")
    event_device = LaunchConfiguration("event_device")
    debug_raw_events = LaunchConfiguration("debug_raw_events")
    joy_deadzone = LaunchConfiguration("joy_deadzone")
    autorepeat_rate = LaunchConfiguration("autorepeat_rate")
    publish_rate_hz = LaunchConfiguration("publish_rate_hz")
    mapper_deadzone = LaunchConfiguration("mapper_deadzone")
    position_velocity_mps = LaunchConfiguration("position_velocity_mps")
    joint3_velocity_radps = LaunchConfiguration("joint3_velocity_radps")
    joint4_velocity_radps = LaunchConfiguration("joint4_velocity_radps")
    left_stick_x_axis = LaunchConfiguration("left_stick_x_axis")
    left_stick_y_axis = LaunchConfiguration("left_stick_y_axis")
    right_stick_joint3_axis = LaunchConfiguration("right_stick_joint3_axis")
    joint4_axis = LaunchConfiguration("joint4_axis")
    joint4_right_button = LaunchConfiguration("joint4_right_button")
    joint4_left_button = LaunchConfiguration("joint4_left_button")
    gripper_open_axis = LaunchConfiguration("gripper_open_axis")
    gripper_close_axis = LaunchConfiguration("gripper_close_axis")
    fixed_x = LaunchConfiguration("fixed_x")
    initial_y = LaunchConfiguration("initial_y")
    initial_z = LaunchConfiguration("initial_z")
    left_stick_x_scale = LaunchConfiguration("left_stick_x_scale")
    left_stick_y_scale = LaunchConfiguration("left_stick_y_scale")
    right_stick_joint3_scale = LaunchConfiguration("right_stick_joint3_scale")
    joint4_axis_scale = LaunchConfiguration("joint4_axis_scale")
    gripper_scale = LaunchConfiguration("gripper_scale")
    trigger_released_value = LaunchConfiguration("trigger_released_value")
    trigger_pressed_value = LaunchConfiguration("trigger_pressed_value")
    ik_publish_rate_hz = LaunchConfiguration("ik_publish_rate_hz")
    ik_time_from_start = LaunchConfiguration("ik_time_from_start")

    controllers_default = PathJoinSubstitution(
        [FindPackageShare("arm_config"), "config", "ros2_controllers.yaml"]
    )
    teleop_rviz_default = PathJoinSubstitution([FindPackageShare("arm_launch"), "rviz", "teleop.rviz"])

    ros2_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("arm_bringup"), "launch", "ros2_control.launch.py"])
        ),
        launch_arguments={
            "controllers_file": controllers_file,
            "use_sim_time": use_sim_time,
            "start_robot_state_publisher": "true",
        }.items(),
    )

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("arm_launch"), "launch", "move_group.launch.py"])
        ),
        condition=IfCondition(start_move_group),
        launch_arguments={
            "controllers_file": controllers_file,
            "use_sim_time": use_sim_time,
            "start_rviz": "false",
        }.items(),
    )

    logitech_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("arm_launch"), "launch", "logitech_ik_teleop.launch.py"])
        ),
        launch_arguments={
            "device_id": device_id,
            "use_event_device": use_event_device,
            "event_device": event_device,
            "debug_raw_events": debug_raw_events,
            "joy_deadzone": joy_deadzone,
            "autorepeat_rate": autorepeat_rate,
            "publish_rate_hz": publish_rate_hz,
            "mapper_deadzone": mapper_deadzone,
            "position_velocity_mps": position_velocity_mps,
            "joint3_velocity_radps": joint3_velocity_radps,
            "joint4_velocity_radps": joint4_velocity_radps,
            "left_stick_x_axis": left_stick_x_axis,
            "left_stick_y_axis": left_stick_y_axis,
            "right_stick_joint3_axis": right_stick_joint3_axis,
            "joint4_axis": joint4_axis,
            "joint4_right_button": joint4_right_button,
            "joint4_left_button": joint4_left_button,
            "gripper_open_axis": gripper_open_axis,
            "gripper_close_axis": gripper_close_axis,
            "fixed_x": fixed_x,
            "initial_y": initial_y,
            "initial_z": initial_z,
            "left_stick_x_scale": left_stick_x_scale,
            "left_stick_y_scale": left_stick_y_scale,
            "right_stick_joint3_scale": right_stick_joint3_scale,
            "joint4_axis_scale": joint4_axis_scale,
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

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="teleop_rviz",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(start_teleop_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "controllers_file",
                default_value=controllers_default,
                description="Controller config shared by ros2_control and MoveIt",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("start_teleop_rviz", default_value="true"),
            DeclareLaunchArgument(
                "start_move_group",
                default_value="false",
                description="Start MoveIt move_group alongside teleop. Disabled by default for smoother RViz teleop.",
            ),
            DeclareLaunchArgument("rviz_config", default_value=teleop_rviz_default),
            DeclareLaunchArgument(
                "automatic_discovery_range",
                default_value="SUBNET",
                description="DDS discovery scope. Use SUBNET for interface-bound setups; LOCALHOST keeps sim local-only.",
            ),
            DeclareLaunchArgument("device_id", default_value="0"),
            DeclareLaunchArgument("use_event_device", default_value="false"),
            DeclareLaunchArgument("event_device", default_value="/dev/input/event0"),
            DeclareLaunchArgument("debug_raw_events", default_value="false"),
            DeclareLaunchArgument("joy_deadzone", default_value="0.05"),
            DeclareLaunchArgument("autorepeat_rate", default_value="50.0"),
            DeclareLaunchArgument("publish_rate_hz", default_value="30.0"),
            DeclareLaunchArgument("mapper_deadzone", default_value="0.12"),
            DeclareLaunchArgument("position_velocity_mps", default_value="0.30"),
            DeclareLaunchArgument("joint3_velocity_radps", default_value="1.20"),
            DeclareLaunchArgument("joint4_velocity_radps", default_value="1.20"),
            DeclareLaunchArgument("left_stick_x_axis", default_value="0"),
            DeclareLaunchArgument("left_stick_y_axis", default_value="1"),
            DeclareLaunchArgument("right_stick_joint3_axis", default_value="4"),
            DeclareLaunchArgument("joint4_axis", default_value="3"),
            DeclareLaunchArgument("joint4_right_button", default_value="5"),
            DeclareLaunchArgument("joint4_left_button", default_value="4"),
            DeclareLaunchArgument("gripper_open_axis", default_value="2"),
            DeclareLaunchArgument("gripper_close_axis", default_value="5"),
            DeclareLaunchArgument("fixed_x", default_value="1.27"),
            DeclareLaunchArgument("initial_y", default_value="0.0"),
            DeclareLaunchArgument("initial_z", default_value="-2.44"),
            DeclareLaunchArgument("left_stick_x_scale", default_value="-1.0"),
            DeclareLaunchArgument("left_stick_y_scale", default_value="1.0"),
            DeclareLaunchArgument("right_stick_joint3_scale", default_value="1.0"),
            DeclareLaunchArgument("joint4_axis_scale", default_value="1.0"),
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
            rviz_node,
        ]
    )
