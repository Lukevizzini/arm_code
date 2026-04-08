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
3) Echo the interpreted outputs:
   - `ros2 topic echo /ik_target_pose`
   - `ros2 topic echo /joint3_command`
   - `ros2 topic echo /joint4_command`
   - `ros2 topic echo /gripper_open_close_command`

Default Logitech mapping matches the common ROS `joy_node` layout:
- Left stick X/Y: updates the end-effector target position in `y/z` while `x` stays fixed.
- Right stick up/down: publishes a scaled command for the third joint from the base (`/joint3_command`).
- Left bumper / right bumper: positive open / negative close gripper command.
- X button / B button: counterclockwise / clockwise command for the last joint (`/joint4_command`).

The published `PoseStamped` can be used directly as a MoveIt IK pose target input if you keep the same `base_link` frame.

## Logitech IK launches
- Sim: `ros2 launch arm_launch sim_logitech_teleop.launch.py`
- Hardware + PWM bridge: `ros2 launch arm_launch hardware_logitech_ik.launch.py`

Both launches now use the shared MoveIt IK path:
- `logitech_to_ik` publishes the Cartesian target plus direct joint commands.
- `logitech_ik_to_trajectory` calls MoveIt's `compute_ik` service and publishes `arm_controller/joint_trajectory`.

## Swap to a real gamepad later
- Replace `keyboard_to_joy` with `ros2 run joy joy_node` and keep `joy_to_trajectory` running; it listens on `/joy`.
- Axis order defaults to `axis_map=[0,1,4,3]` for (joint1..4). Change via parameters if your gamepad order differs.
