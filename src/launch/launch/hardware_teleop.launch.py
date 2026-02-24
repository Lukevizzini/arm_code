from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    controllers_file = LaunchConfiguration("controllers_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_pwm_bridge = LaunchConfiguration("start_pwm_bridge")
    rc_channels = LaunchConfiguration("rc_channels")
    rc_override_topic = LaunchConfiguration("rc_override_topic")
    start_mavros = LaunchConfiguration("start_mavros")
    fcu_url = LaunchConfiguration("fcu_url")
    gcs_url = LaunchConfiguration("gcs_url")
    mavros_pluginlists_yaml = LaunchConfiguration("mavros_pluginlists_yaml")
    require_start_trigger = LaunchConfiguration("require_start_trigger")
    start_trigger_topic = LaunchConfiguration("start_trigger_topic")

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("arm_bringup"), "launch", "ros2_control.launch.py"])
        ),
        launch_arguments={"controllers_file": controllers_file, "use_sim_time": use_sim_time}.items(),
    )

    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("arm_launch"), "launch", "keyboard_teleop.launch.py"])
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    pwm_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("arm_pwm_bridge"), "launch", "pwm_bridge.launch.py"])
        ),
        launch_arguments={
            "rc_channels": rc_channels,
            "rc_override_topic": rc_override_topic,
            "start_mavros": start_mavros,
            "fcu_url": fcu_url,
            "gcs_url": gcs_url,
            "mavros_pluginlists_yaml": mavros_pluginlists_yaml,
            "require_start_trigger": require_start_trigger,
            "start_trigger_topic": start_trigger_topic,
        }.items(),
        condition=IfCondition(start_pwm_bridge),
    )

    controllers_default = PathJoinSubstitution([FindPackageShare("arm_config"), "config", "ros2_controllers.yaml"])

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "controllers_file",
                default_value=controllers_default,
                description="ros2_control controller configuration",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "start_pwm_bridge",
                default_value="true",
                description="Start the JointTrajectory->PWM bridge alongside teleop",
            ),
            DeclareLaunchArgument("rc_channels", default_value="[12,13,14,15,16]"),
            DeclareLaunchArgument("rc_override_topic", default_value="/uas1/mavros/rc/override"),
            DeclareLaunchArgument("start_mavros", default_value="false"),
            DeclareLaunchArgument("fcu_url", default_value="udp://:14550@"),
            DeclareLaunchArgument("gcs_url", default_value=""),
            DeclareLaunchArgument(
                "mavros_pluginlists_yaml",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("arm_pwm_bridge"), "config", "mavros_pluginlists.yaml"]
                ),
            ),
            DeclareLaunchArgument("require_start_trigger", default_value="false"),
            DeclareLaunchArgument("start_trigger_topic", default_value="/arm_pwm_bridge/start"),
            bringup_launch,
            teleop_launch,
            pwm_bridge_launch,
        ]
    )
