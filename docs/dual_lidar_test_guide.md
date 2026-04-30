# 双 MID-360 与串口测试指南

## 当前检测结果

```text
主网卡: enp86s0 = 192.168.1.2/24
主雷达: 192.168.1.173, MAC e4:7a:2c:98:9f:21, ping 正常
第二雷达: 192.168.1.166, MAC 48:1c:b9:e3:38:47, ping 正常
未发现 192.168.1.174
```

主雷达仍使用 `src/rm_bringup/config/pb2025_xmu_mid360_config.json`，第二雷达使用 `src/rm_bringup/config/pb2025_xmu_second_mid360_config.json`。第二雷达只用于 `/second_lidar_obstacle_cloud`，不进入 Point-LIO、建图或 small_gicp。

## 串口绑定

当前检测到：

```text
/dev/ttyUSB0: CH340, USB path 3-8, 460800 下有下位机回包
/dev/ttyUSB1: CH340, USB path 3-9, 未读到下位机回包
```

安装 udev 规则：

```bash
cd /home/rm/Desktop/SENTRY_FULL/XMU_RCS_SENTRY
sudo cp docs/99-rm-serial.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger --action=add --subsystem-match=tty --sysname-match=ttyUSB0 --settle
ls -l /dev/rm_serial
```

如果 `/dev/rm_serial` 没出现，拔插下位机 USB 或重启后再看：

```bash
sudo udevadm test /sys/class/tty/ttyUSB0 2>&1 | grep rm_serial
ls -l /dev/ttyUSB* /dev/serial/by-path/
udevadm info -q property -n /dev/ttyUSB0
```

临时验证可以先手动建软链接：

```bash
sudo ln -sf ttyUSB0 /dev/rm_serial
```

## 基础检查

```bash
ip -br addr
ip neigh show dev enp86s0
ping -c 1 192.168.1.173
ping -c 1 192.168.1.166
```

确认配置：

```bash
grep -n '"ip"' src/rm_bringup/config/pb2025_xmu_mid360_config.json
grep -n '"ip"' src/rm_bringup/config/pb2025_xmu_second_mid360_config.json
```

## 单雷达主链测试

先不启用第二雷达，确认 PB2025 主导航没有被影响：

```bash
cd /home/rm/Desktop/SENTRY_FULL/XMU_RCS_SENTRY
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch rm_bringup sentry_bringup.launch.py \
  navigation_only:=true \
  use_serial:=true \
  serial_device:=/dev/rm_serial \
  baudrate:=460800 \
  enable_navigation:=true \
  slam:=False \
  map:=/home/rm/Desktop/SENTRY_FULL/maps/self_filtered_map.yaml \
  enable_small_gicp:=false \
  use_rviz:=false
```

检查：

```bash
ros2 topic hz /odometry
ros2 topic hz /terrain_map
ros2 topic hz /cmd_vel
ros2 topic hz /nav_cmd
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
```

## 第二雷达避障测试

第二雷达只用于近场 obstacle cloud：

```bash
ros2 launch rm_bringup sentry_bringup.launch.py \
  navigation_only:=true \
  use_serial:=true \
  serial_device:=/dev/rm_serial \
  baudrate:=460800 \
  enable_navigation:=true \
  slam:=False \
  map:=/home/rm/Desktop/SENTRY_FULL/maps/self_filtered_map.yaml \
  enable_small_gicp:=false \
  enable_second_lidar_obstacle:=true \
  use_rviz:=false
```

检查：

```bash
ros2 run tf2_ros tf2_echo base_link second_mid360
ros2 topic list | grep second
ros2 topic hz /second_livox/lidar
ros2 topic hz /second_lidar_obstacle_cloud
ros2 topic echo /second_lidar_obstacle_debug
```

预期：

```text
/second_livox/lidar 有频率
/second_lidar_obstacle_cloud 有频率
/second_lidar_obstacle_debug 中 dropped_tf_count 不持续快速增长
/odometry 仍来自主雷达 Point-LIO
```

## 到点与 nav_cmd 检查

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{
  pose: {
    header: {frame_id: 'map'},
    pose: {
      position: {x: 1.0, y: 0.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  }
}"
ros2 topic echo /nav_cmd
```

到点后应看到：

```text
linear_x = 0
linear_y = 0
angular_z = 0
is_reached = 1
```

## 常见问题

- `/dev/rm_serial` 不存在：udev 规则未安装、未 reload，或下位机插到了不同 USB 物理口。重新查看 `/dev/serial/by-path/` 并更新 `docs/99-rm-serial.rules` 里的 `KERNELS=="3-8:1.0"`。
- 第二雷达没点云：确认 `192.168.1.166` 能 ping 通，确认 `pb2025_xmu_second_mid360_config.json` 中 host IP 是 `192.168.1.2`。
- 坡道误判加重：先用 `enable_second_lidar_obstacle:=false` 做对照，再考虑提高第二雷达过滤节点 `min_height` 或缩小 `max_range`。
