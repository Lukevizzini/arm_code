from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    controllers_file = LaunchConfiguration("controllers_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_rviz = LaunchConfiguration("start_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

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

    controllers_default = PathJoinSubstitution([FindPackageShare("arm_config"), "config", "ros2_controllers.yaml"])
    teleop_rviz_default = PathJoinSubstitution([FindPackageShare("arm_launch"), "rviz", "teleop.rviz"])
    description_path = PathJoinSubstitution([FindPackageShare("arm_description"), "urdf", "four_dof_arm.urdf.xacro"])
    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", description_path, " controllers_file:=", controllers_file]),
            value_type=str,
        ),
        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
    }

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

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
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

    ik_node = Node(
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
        parameters=[robot_description],
        arguments=["-d", rviz_config],
        condition=IfCondition(start_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "controllers_file",
                default_value=controllers_default,
                description="Controller config shared by ros2_control and MoveIt",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("start_rviz", default_value="false"),
            DeclareLaunchArgument("rviz_config", default_value=teleop_rviz_default),
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
            DeclareLaunchArgument("right_stick_joint3_scale", default_value="-1.0"),
            DeclareLaunchArgument("joint4_axis_scale", default_value="-1.0"),
            DeclareLaunchArgument("gripper_scale", default_value="-1.0"),
            DeclareLaunchArgument("trigger_released_value", default_value="1.0"),
            DeclareLaunchArgument("trigger_pressed_value", default_value="-1.0"),
            DeclareLaunchArgument("ik_publish_rate_hz", default_value="15.0"),
            DeclareLaunchArgument("ik_time_from_start", default_value="0.12"),
            ros2_control_launch,
            robot_state_publisher_node,
            logitech_launch,
            ik_node,
            rviz_node,
        ]
    )
