# Arm Control Workspace

## Host + Raspberry Pi (split setup)
Use this when keyboard teleop runs on the host computer and PWM bridge runs on the Raspberry Pi.

### 1) Set ROS networking on **both** machines
Run this in every terminal you will use:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/arm_v2/install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
```

Important:
- Use the same `ROS_DOMAIN_ID` value on both machines.
- Both machines must be on the same LAN/subnet.

### 2) Host computer: start keyboard teleop
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/arm_v2/install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
ros2 launch arm_launch keyboard_teleop.launch.py use_sim_time:=false
```

### 3) Raspberry Pi: start PWM bridge (+ MAVROS)
If MAVROS is not already running:
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/arm_v2/install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
ros2 launch arm_pwm_bridge pwm_bridge.launch.py \
  start_mavros:=true \
  fcu_url:=serial:///dev/ttyACM0:115200 \
  rc_override_topic:=/uas1/mavros/rc/override
```

If MAVROS is already running separately on the Pi:
```bash
ros2 launch arm_pwm_bridge pwm_bridge.launch.py \
  start_mavros:=false \
  rc_override_topic:=/uas1/mavros/rc/override
```

### 4) Optional arm trigger
If you launch with `require_start_trigger:=true`, use:
```bash
ros2 topic pub --once /arm_pwm_bridge/start std_msgs/msg/Bool "{data: true}"   # ARM (start sending RC overrides)
ros2 topic pub --once /arm_pwm_bridge/start std_msgs/msg/Bool "{data: false}"  # DISARM (pause RC override sending)
```

### 5) Quick cross-machine checks
On Raspberry Pi:
```bash
ros2 topic echo /arm_controller/joint_trajectory --once
ros2 topic info /uas1/mavros/rc/override
```

You should see:
- A trajectory message arriving from the host.
- At least one subscriber on `/uas1/mavros/rc/override` (MAVROS).

## Package READMEs
- `src/teleop/README.md`
- `src/pwm_bridge/README.md`
- `src/description/README.md`
