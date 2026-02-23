# arm_pwm_bridge

`JointTrajectory` -> PWM bridge for the 4-DOF arm using MAVROS RC override
(`mavros_msgs/msg/OverrideRCIn`).
It listens on `arm_controller/joint_trajectory` (what `joy_to_trajectory` publishes)
and commands Navigator PWM outputs (for example channels `9-16`).

## Prereqs
- Install MAVROS and message definitions:
  ```bash
  sudo apt install -y ros-$ROS_DISTRO-mavros ros-$ROS_DISTRO-mavros-msgs
  ```
- Start MAVROS so `/mavros/rc/override` is available.
- Ensure only your target outputs (for example 11-15) are configured for RC passthrough mapping.

## Build
```bash
cd /path/to/arm_control
rosdep install --from-paths src -y --ignore-src
colcon build --packages-select arm_pwm_bridge arm_teleop
source install/setup.bash
```

## Run on hardware (no Gazebo)
Terminal 1: ros2_control bringup (controllers + state publisher):
```bash
ros2 launch arm_bringup ros2_control.launch.py
```

Terminal 2: keyboard/gamepad teleop to publish joint targets:
```bash
ros2 launch arm_launch keyboard_teleop.launch.py use_sim_time:=false
```

Terminal 3: PWM bridge:
```bash
ros2 launch arm_pwm_bridge pwm_bridge.launch.py \
  pwm_channels:=[11,12,13,14,15] \
  override_topic:=/mavros/rc/override \
  angle_min_rad:=[-2.6,-2.0,-2.6,-2.6,-1.0] \
  angle_max_rad:=[2.6,2.6,2.6,2.6,1.0] \
  pulse_min_us:=700.0 pulse_max_us:=2300.0
```

## Parameters (node or launch override)
- `trajectory_topic` (string): topic to listen for JointTrajectory commands (default `arm_controller/joint_trajectory`).
- `joint_names` (string list): joints to map, order aligned with channels.
- `pwm_channels` (int list): output channels to drive (Navigator supports `1..16`).
- `angle_min_rad` / `angle_max_rad` (float list): clamp ranges per joint.
- `pulse_min_us` / `pulse_max_us` (float): microsecond range sent to servos.
- `initial_positions_rad` (float list): optional starting setpoint.
- `override_topic` (string): MAVROS override topic (default `/mavros/rc/override`).
- `gripper_joy_topic` (string): Joy topic used for gripper buttons (default `joy`).
- `gripper_joint_name` (string): joint in `joint_names` that should be gripper-controlled (default `gripper_joint`).
- `gripper_open_button_index` / `gripper_close_button_index` (int): Joy button indices for open/close (defaults `5`/`4` -> RB/LB).
- `gripper_step_rad` (float): gripper angle step applied per Joy message while the button is held.
- `send_neutral_on_shutdown` (bool): send neutral PWM to controlled channels when shutting down.
- `neutral_pwm` (int): neutral PWM value used on shutdown.
- `publish_joint_states` (bool): republish the last commanded angles for RViz.

Notes:
- This bridge suits the simple `joy_to_trajectory` path. It does not implement FollowJointTrajectory actions (used by MoveIt); add a full ros2_control hardware interface if you need that.
- RC override is sent only on configured channels (`pwm_channels`), while all other channels are set to `CHAN_NOCHANGE`.
- With default keyboard teleop, `u` (LB) closes and `o` (RB) opens the gripper channel on RC15.
