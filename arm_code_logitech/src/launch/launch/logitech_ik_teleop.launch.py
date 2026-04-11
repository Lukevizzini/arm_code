from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
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

    return LaunchDescription(
        [
            DeclareLaunchArgument("device_id", default_value="0", description="Joystick device index for joy_node"),
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
            Node(
                package="joy",
                executable="joy_node",
                name="logitech_joy",
                output="screen",
                condition=UnlessCondition(use_event_device),
                parameters=[
                    {
                        "device_id": ParameterValue(device_id, value_type=int),
                        "deadzone": ParameterValue(joy_deadzone, value_type=float),
                        "autorepeat_rate": ParameterValue(autorepeat_rate, value_type=float),
                    }
                ],
            ),
            Node(
                package="arm_teleop",
                executable="event_to_joy",
                name="event_to_joy",
                output="screen",
                condition=IfCondition(use_event_device),
                parameters=[
                    {
                        "event_device": ParameterValue(event_device, value_type=str),
                        "deadzone": ParameterValue(joy_deadzone, value_type=float),
                        "publish_rate_hz": ParameterValue(autorepeat_rate, value_type=float),
                        "debug_raw_events": ParameterValue(debug_raw_events, value_type=bool),
                    }
                ],
            ),
            Node(
                package="arm_teleop",
                executable="logitech_to_ik",
                name="logitech_to_ik",
                output="screen",
                parameters=[
                    {
                        "publish_rate_hz": ParameterValue(publish_rate_hz, value_type=float),
                        "deadzone": ParameterValue(mapper_deadzone, value_type=float),
                        "position_velocity_mps": ParameterValue(position_velocity_mps, value_type=float),
                        "joint3_velocity_radps": ParameterValue(joint3_velocity_radps, value_type=float),
                        "joint4_velocity_radps": ParameterValue(joint4_velocity_radps, value_type=float),
                        "left_stick_x_axis": ParameterValue(left_stick_x_axis, value_type=int),
                        "left_stick_y_axis": ParameterValue(left_stick_y_axis, value_type=int),
                        "right_stick_joint3_axis": ParameterValue(right_stick_joint3_axis, value_type=int),
                        "joint4_axis": ParameterValue(joint4_axis, value_type=int),
                        "joint4_right_button": ParameterValue(joint4_right_button, value_type=int),
                        "joint4_left_button": ParameterValue(joint4_left_button, value_type=int),
                        "gripper_open_axis": ParameterValue(gripper_open_axis, value_type=int),
                        "gripper_close_axis": ParameterValue(gripper_close_axis, value_type=int),
                        "fixed_x": ParameterValue(fixed_x, value_type=float),
                        "initial_y": ParameterValue(initial_y, value_type=float),
                        "initial_z": ParameterValue(initial_z, value_type=float),
                        "left_stick_x_scale": ParameterValue(left_stick_x_scale, value_type=float),
                        "left_stick_y_scale": ParameterValue(left_stick_y_scale, value_type=float),
                        "right_stick_joint3_scale": ParameterValue(right_stick_joint3_scale, value_type=float),
                        "joint4_axis_scale": ParameterValue(joint4_axis_scale, value_type=float),
                        "gripper_scale": ParameterValue(gripper_scale, value_type=float),
                        "trigger_released_value": ParameterValue(trigger_released_value, value_type=float),
                        "trigger_pressed_value": ParameterValue(trigger_pressed_value, value_type=float),
                    }
                ],
            ),
        ]
    )
