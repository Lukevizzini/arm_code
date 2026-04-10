from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    controllers_file = LaunchConfiguration("controllers_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_robot_state_publisher = LaunchConfiguration("start_robot_state_publisher")

    description_path = PathJoinSubstitution([FindPackageShare("arm_description"), "urdf", "four_dof_arm.urdf.xacro"])
    controllers_default = PathJoinSubstitution([FindPackageShare("arm_config"), "config", "ros2_controllers.yaml"])

    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", description_path, " controllers_file:=", controllers_file]),
            value_type=str,
        ),
        "use_sim_time": use_sim_time,
    }

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[robot_description, controllers_file],
        sigterm_timeout="2",
        sigkill_timeout="5",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
        condition=IfCondition(start_robot_state_publisher),
        sigterm_timeout="2",
        sigkill_timeout="5",
    )

    joint_state_broadcaster_loader = ExecuteProcess(
        output="screen",
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "joint_state_broadcaster",
            "--set-state",
            "active",
            "-c",
            "/controller_manager",
        ],
    )

    arm_controller_loader = ExecuteProcess(
        output="screen",
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "arm_controller",
            "--set-state",
            "active",
            "-c",
            "/controller_manager",
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "controllers_file",
                default_value=controllers_default,
                description="ros2_control controller configuration",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false", description="Use simulation clock"),
            DeclareLaunchArgument(
                "start_robot_state_publisher",
                default_value="true",
                description="Start robot_state_publisher alongside ros2_control",
            ),
            control_node,
            robot_state_publisher,
            joint_state_broadcaster_loader,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_broadcaster_loader,
                    on_exit=[arm_controller_loader],
                )
            ),
        ]
    )
