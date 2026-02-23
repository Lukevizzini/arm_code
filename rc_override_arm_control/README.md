# rc_override_arm_control

ROS 2 workspace for the 4-DOF arm with `ros2_control`, keyboard teleop, and MAVROS RC override output for Navigator PWM channels.

This workspace is configured for RC override mapping on channels `11-15` by default
(joint1-4 + gripper on RC15).

## Build
```bash
cd /Users/eligriffin/Desktop/rc_override_arm_control
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
source install/setup.bash
```

## Prereqs (hardware PWM path)
Install MAVROS packages on the machine running the PWM bridge:
```bash
sudo apt install -y ros-$ROS_DISTRO-mavros ros-$ROS_DISTRO-mavros-msgs
```

Start MAVROS so `/mavros/rc/override` is available:
```bash
ros2 launch mavros apm.launch fcu_url:=udp://@127.0.0.1:14550
```

## Run: teleop + PWM (single machine)
One-shot launch (ros2_control bringup + keyboard teleop + PWM bridge):
```bash
ros2 launch arm_launch hardware_teleop.launch.py
```

Useful overrides:
- Disable PWM bridge: `start_pwm_bridge:=false`
- Change channels: `pwm_channels:=[11,12,13,14,15]`
- Change override topic: `override_topic:=/mavros/rc/override`

## Run: split Host + Pi (recommended)
Use this when keyboard teleop runs on your host computer and MAVROS/PWM runs on the Pi.

Set matching ROS networking vars on both:
```bash
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Pi terminal(s):
```bash
# Terminal 1: MAVROS
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/rc_override_arm_control
source install/setup.bash
ros2 launch mavros apm.launch fcu_url:=udp://@127.0.0.1:14550
```

```bash
# Terminal 2: PWM bridge
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/rc_override_arm_control
source install/setup.bash
ros2 launch arm_pwm_bridge pwm_bridge.launch.py \
  pwm_channels:=[11,12,13,14,15] \
  override_topic:=/mavros/rc/override
```

Host terminal:
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd /Users/eligriffin/Desktop/rc_override_arm_control
source install/setup.bash
ros2 launch arm_launch keyboard_teleop.launch.py use_sim_time:=false velocity_scale:=1.2
```

## Quick verification
On Pi:
```bash
ros2 topic echo /mavros/state --once
ros2 topic echo /mavros/rc/override
```

On host:
```bash
ros2 topic hz /arm_controller/joint_trajectory
```

Keyboard gripper control:
- `u` (LB) closes gripper
- `o` (RB) opens gripper
- These buttons drive RC15 through the PWM bridge (MAVROS override).

## MoveIt (optional planning)
```bash
ros2 launch arm_launch move_group.launch.py start_rviz:=true
```

## Package map
- `src/description`: URDF/XACRO + meshes
- `src/config`: controller + MoveIt configuration
- `src/bringup`: `ros2_control` bringup launch
- `src/launch`: convenience launch files
- `src/teleop`: keyboard-to-joy and joy-to-trajectory
- `src/pwm_bridge`: `JointTrajectory` -> MAVROS RC override bridge

## Notes
- The PWM bridge sends RC override only on configured channels; all other channels are `CHAN_NOCHANGE`.
- `ros2_control` in this workspace uses `mock_components/GenericSystem` for controller hosting; hardware PWM actuation is done by `arm_pwm_bridge` through MAVROS.
