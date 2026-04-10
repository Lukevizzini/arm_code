# arm_teleop

Teleop helpers for the 4-DOF arm. The package includes a keyboard-to-joy shim for joint-space testing and a Logitech gamepad mapper that publishes an end-effector IK target plus direct joint/gripper commands from `/joy`.

## Key mappings
- `w/s`: left stick up/down (joint2)
- `a/d`: left stick left/right (joint1)
- `i/k`: right stick up/down (joint3)
- `j/l`: right stick left/right (joint4)
- `space`: A button, `m`: B button, `n`: Y button, `u/o`: LB/RB
- Hold a key to keep a deflection (OS key repeat keeps the axis nonzero).

## Typical run
1) Build and source the workspace: `colcon build --symlink-install && source install/setup.bash`.
2) Bring up ros2_control (controllers + robot_state_publisher):  
   `ros2 launch arm_bringup ros2_control.launch.py`
3) Start the PWM bridge if you're driving real servos:  
   `ros2 launch arm_pwm_bridge pwm_bridge.launch.py`
4) Start the keyboard teleop stack (Joy publisher + joint trajectory mapper):  
   `ros2 launch arm_launch keyboard_teleop.launch.py velocity_scale:=1.2`
   - Adjust stick speed with `velocity_scale:=<rad_per_sec>` and deadzone with `deadzone:=<value>`.
   - B button (`m`) resets joint targets to zero.

## Logitech controller -> IK target echo
1) Plug in the controller.
2) Launch the joystick driver and mapper:
   `ros2 launch arm_launch logitech_ik_teleop.launch.py`
   - If a stick direction is inverted on your controller/OS, flip it with a scale override such as `left_stick_y_scale:=-1.0`.
   - If axis indices differ on your controller, override them at launch (example: `left_stick_x_axis:=0 left_stick_y_axis:=1 right_stick_joint3_axis:=3 gripper_open_axis:=2 gripper_close_axis:=5`).
3) Echo the interpreted outputs:
   - `ros2 topic echo /ik_target_pose`
   - `ros2 topic echo /joint3_command`
   - `ros2 topic echo /joint4_command`
   - `ros2 topic echo /gripper_open_close_command`

Default Logitech mapping matches the common ROS `joy_node` layout:
- Left stick X/Y: updates the end-effector target position in `y/z` while `x` stays fixed.
- Right stick up/down: publishes a scaled command for the third joint from the base (`/joint3_command`).
- Left bumper / right bumper: counterclockwise / clockwise command for the last joint (`/joint4_command`).
- Trigger axes: differential open/close command for the gripper (`/gripper_open_close_command`).

The published `PoseStamped` can be used directly as a MoveIt IK pose target input if you keep the same `base_link` frame.

## Logitech IK launches
- Sim: `ros2 launch arm_launch sim_logitech_teleop.launch.py`
- Hardware + PWM bridge: `ros2 launch arm_launch hardware_logitech_ik.launch.py`

If your shell exports `ROS_NETWORK_INTERFACES` (for example `eth0`), prefer `automatic_discovery_range:=SUBNET` in sim launch to avoid hiding topics from CLI discovery.

Both launches now use the shared MoveIt IK path:
- `logitech_to_ik` publishes the Cartesian target plus direct joint commands.
- `logitech_ik_to_trajectory` calls MoveIt's `compute_ik` service and publishes `arm_controller/joint_trajectory`.

## Swap to a real gamepad later
- Replace `keyboard_to_joy` with `ros2 run joy joy_node` and keep `joy_to_trajectory` running; it listens on `/joy`.
- Axis order defaults to `axis_map=[0,1,4,3]` for (joint1..4). Change via parameters if your gamepad order differs.

## Logitech F310 On WSL/Linux
If the F310 does not publish `/joy` through `joy_node`, use this checklist:

1) Put the physical switch on the back of the controller in `D` mode.
2) Replug the controller.
3) Confirm Linux sees it:
   - `lsusb`
   - `ls /dev/input`
   - `cat /proc/bus/input/devices`
4) If Linux only exposes `/dev/input/event0` and not `/dev/input/js0`, use the built-in event fallback instead of `joy_node`.

### Temporary permission fix
If the event fallback fails with `Permission denied: '/dev/input/event0'`, allow read access:

`sudo chmod a+r /dev/input/event0`

More permanent options:
- Add your user to the `input` group: `sudo usermod -aG input $USER`
- Or grant ACL access: `sudo setfacl -m u:$USER:r /dev/input/event0`

### Test controller input only
Build and source first:

`colcon build --symlink-install && source install/setup.bash`

Run the Logitech teleop input stack using the Linux event device:

`ros2 launch arm_launch logitech_ik_teleop.launch.py use_event_device:=true event_device:=/dev/input/event0`

In another terminal, verify:
- `ros2 topic echo /joy`
- `ros2 topic echo /joint3_command`
- `ros2 topic echo /joint4_command`

### Sim launch with event-device fallback
Use the same controller path in sim:

`ros2 launch arm_launch sim_logitech_teleop.launch.py use_event_device:=true event_device:=/dev/input/event0`

### Hardware launch with event-device fallback
Use the same controller path on hardware:

`ros2 launch arm_launch hardware_logitech_ik.launch.py use_event_device:=true event_device:=/dev/input/event0`

### Useful launch overrides
Keep overrides on the same command line:

`ros2 launch arm_launch hardware_logitech_ik.launch.py use_event_device:=true event_device:=/dev/input/event0 left_stick_y_scale:=-1.0 right_stick_joint3_scale:=1.0`

Common fixes:
- Flip vertical stick direction: `left_stick_y_scale:=-1.0`
- Flip joint3 stick direction: `right_stick_joint3_scale:=1.0`
- Force alternate right-stick axis: `right_stick_joint3_axis:=3`

### What to check when nothing moves
- If `/joy` is flat: the controller input path is still broken.
- If `/joy` moves but `/joint3_command` stays zero: the axis map is wrong.
- If `/joint3_command` moves but the arm does not: the downstream hardware or controller bridge is the problem.
