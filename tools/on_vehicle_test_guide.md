# On-Vehicle Test Guide -- Serial, Bringup & Rosbag Recording

## Step 0: Build

```bash
cd ~/Desktop/SENTRY_FULL/XMU_RCS_SENTRY
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Step 1: Test Serial Communication (CRITICAL)

### 1.1 Check serial device
```bash
ls -l /dev/rm_serial /dev/serial/by-id/ /dev/ttyUSB* /dev/ttyACM*
# Expected: /dev/rm_serial -> /dev/ttyUSBx
```

### 1.2 Fix permissions
```bash
sudo chmod 666 /dev/rm_serial
# OR permanent: sudo usermod -aG dialout $USER  (then re-login)
```

### 1.3 Verify baudrate matches STM32
Default: 460800 bps. Check Control_STM/STM32F405/usart.cpp for STM32 config.

### 1.4 Start hw_bridge alone
```bash
source install/setup.bash
ros2 run rm_hw_bridge hw_bridge_node --ros-args \
  -p serial_device:=/dev/rm_serial \
  -p baudrate:=460800 \
  -p require_crc_check:=false
```
Expected output:
```
[rm_hw_bridge] Opening serial: /dev/rm_serial @ 460800 bps
[rm_hw_bridge] rm_hw_bridge node initialized (timer-driven publish @ 1kHz).
```

### 1.5 Verify uplink (STM32 -> NUC)
New terminal:
```bash
source install/setup.bash
ros2 topic echo /imu/data --once       # Should show quaternion data
ros2 topic echo /gimbal_status --once  # Should show yaw/pitch/bullet_speed
ros2 topic hz /imu/data               # Should be ~900-1100 Hz
```

| Symptom | Cause | Fix |
|---------|-------|-----|
| No data at all | STM32 off / TX-RX swapped | Check power & wiring |
| Very low freq (~10Hz) | Baudrate mismatch | Check STM32 UART config |
| CRC warnings only | STM32 CRC not implemented | Normal in debug mode |
| yaw_TJ = 0 static | STM32 not in autoaim mode | Check RC switch / mode |

### 1.6 Verify downlink (NUC -> STM32)
```bash
ros2 topic pub --once /gimbal_cmd rm_interfaces/msg/GimbalCmd \
  "{mode: 1, target_yaw: 0.0, target_pitch: 0.0, yaw_vel: 0.0, pitch_vel: 0.0, fire_control: 0, state_switch: 0}"
```
Watch for gimbal twitch = downlink OK.

## Step 2: Test Camera Alone

```bash
source install/setup.bash
ros2 run rm_hik_driver hik_camera_node --ros-args \
  --params-file install/rm_hik_driver/share/rm_hik_driver/config/params.yaml
```
Verify:
```bash
ros2 topic hz /camera/image   # Should show ~30-60 Hz
rqt_image_view                # Should show live camera feed
```

## Step 3: Full Bringup

After both serial and camera verified:
```bash
source install/setup.bash
ros2 launch rm_bringup sentry_bringup.launch.py \
  use_serial:=true \
  serial_device:=/dev/rm_serial \
  baudrate:=460800 \
  target_color:=red \
  publish_debug_image:=true
```

Timeline:
```
T+0.0s: rm_hw_bridge    (serial connect)
T+2.0s: rm_hik_driver   (camera start)
T+4.0s: rm_vision       (YOLO model load)
T+4.0s: rm_autoaim      (tracker + aimer)
```

### Verify full chain
```bash
ros2 topic hz /detector/armors        # ~30Hz detection
ros2 topic echo /gimbal_cmd --once    # Autoaim output (changing values)
ros2 topic echo /gimbal_status --once # STM32 feedback
```

## Step 4: Record Rosbag

```bash
mkdir -p ~/bag2 && cd ~/bag2
ros2 bag record \
  -o autoaim_$(date +%Y%m%d_%H%M%S) \
  /detector/armors \
  /autoaim/debug_world_points \
  /autoaim/gimbal_cmd \
  /autoaim/model_probabilities \
  /gimbal_status \
  /imu/data \
  /joint_states \
  /tf
```

Do NOT record /camera/image (~30MB/s, 1.8GB per minute).

### Target maneuver protocol (60-90 seconds)
| Phase | Action | Duration | Purpose |
|-------|--------|----------|---------|
| A | Stationary | 5s | Baseline |
| B | Constant velocity lateral | 10s | CV model test |
| C | Hard stop / start | 10s | Response speed |
| D | Spinning (>=2 rad/s) | 15s | CTRV model (CORE!) |
| E | Mixed random | 15s | IMM switching |

### After recording
```bash
ros2 bag info ~/bag2/autoaim_XXXXXXXX_XXXXXX
```

## Step 5: Offline Playback & Tuning

```bash
ros2 bag play ~/bag2/autoaim_XXXXXXXX_XXXXXX --clock
python3 tools/tune_ukf.py --bag ~/bag2/.../xxx.db3 --quick
```

## Quick Reference Commands

```bash
# Build
cd ~/Desktop/SENTRY_FULL/XMU_RCS_SENTRY && \
source /opt/ros/humble/setup.bash && colcon build --symlink-install && source install/setup.bash

# Serial only
ros2 run rm_hw_bridge hw_bridge_node --ros-args -p serial_device:=/dev/rm_serial -p require_crc_check:=false

# Full bringup
ros2 launch rm_bringup sentry_bringup.launch.py use_serial:=true serial_device:=/dev/rm_serial target_color:=red publish_debug_image:=true

# Record
mkdir -p ~/bag2 && cd ~/bag2 && ros2 bag record -o autoaim_$(date +%Y%m%d_%H%M%S) /detector/armors /autoaim/debug_world_points /gimbal_status /imu/data /joint_states /tf

# Monitor
ros2 topic hz /imu/data
ros2 topic hz /detector/armors
ros2 topic echo /gimbal_cmd --once
ros2 topic echo /gimbal_status --once
```
