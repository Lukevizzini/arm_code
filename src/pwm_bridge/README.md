# arm_pwm_bridge

`JointTrajectory` -> PWM bridge for the 4-DOF arm using MAVROS RC override.
It listens on `arm_controller/joint_trajectory` (what `joy_to_trajectory` publishes)
and commands Navigator PWM outputs (for example channels `9-16`).
By default it keeps a 5th mapping for `joint5` (commonly used as gripper).

## Prereqs
- Install MAVROS and message definitions:
  ```bash
  sudo apt install -y ros-$ROS_DISTRO-mavros ros-$ROS_DISTRO-mavros-msgs
  ```
- Start MAVROS so `mavros/rc/override` is bridged to the FCU.
- Ensure output channels are configured in ArduPilot (`SERVOx_FUNCTION`) for direct servo control.

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
  rc_channels:=[9,10,11,12,13] \
  angle_min_rad:=[-2.6,-2.0,-2.6,-2.6,-1.5] \
  angle_max_rad:=[2.6,2.6,2.6,2.6,1.5] \
  pulse_min_us:=700.0 pulse_max_us:=2300.0
```

## Parameters (node or launch override)
- `trajectory_topic` (string): topic to listen for JointTrajectory commands (default `arm_controller/joint_trajectory`).
- `joint_names` (string list): joints to map, order aligned with channels.
- `rc_channels` (int list): output channels to drive (Navigator supports `1..16`).
- `angle_min_rad` / `angle_max_rad` (float list): clamp ranges per joint.
- `pulse_min_us` / `pulse_max_us` (float): microsecond range sent to servos.
- `initial_positions_rad` (float list): optional starting setpoint.

Notes:
- This bridge suits the simple `joy_to_trajectory` path. It does not implement FollowJointTrajectory actions (used by MoveIt); add a full ros2_control hardware interface if you need that.
- Each command is sent as `mavros/rc/override` channel values. Adjust ranges to your servo geometry before use.
