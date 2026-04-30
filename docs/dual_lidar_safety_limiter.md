# Dual Lidar Safety Limiter

当前 PB2025 主雷达链路已经实车验证稳定，因此第二雷达第一阶段只作为速度安全壳，不进入导航主链。

## 设计原因

- 单雷达默认模式下，主雷达 `/livox/lidar` 和 `/livox/imu` 仍是 Point-LIO、PB terrain、Nav2 的稳定输入。
- 没有第二网卡时，两个 Livox driver 进程会在同一网段抢设备控制权，所以双雷达 safety 模式改为单个 Livox driver 进程。
- 双雷达模式使用 `multi_topic=1` 和 `xfer_format=1`，按 IP 分流 CustomMsg；Point-LIO 只订阅主雷达 IP 对应 topic。
- 第二雷达直接写入 costmap 时，云台/车体姿态误差容易制造近场鬼障碍。
- 因此第二雷达只转成 `/second_livox/lidar`，滤波为近场障碍云，再对 `/cmd_vel` 做限速，输出 `/cmd_vel_safe`。

## 数据流

```text
livox_ros_driver2 (one process, dual MID-360 config)
  -> /livox/lidar_192_168_1_173 + /livox/imu_192_168_1_173 -> Point-LIO
  -> /livox/lidar_192_168_1_166
  -> livox_custom_to_pointcloud2.py
  -> /second_livox/lidar (PointCloud2, frame_id=second_mid360)
  -> second_lidar_obstacle_filter
  -> /second_lidar_obstacle_cloud (PointCloud2, frame_id=gimbal_yaw)
  -> second_lidar_safety_limiter
  -> /cmd_vel_safe
  -> pb_cmd_vel_to_nav_cmd.py
  -> /nav_cmd
```

默认 `enable_second_lidar_safety:=false` 时，上述节点和 `base_link -> second_mid360` 静态 TF 都不会启动，PB2025 单雷达链路保持稳定版行为。

## 启动

单雷达稳定模式：

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
  enable_second_lidar_safety:=false \
  use_rviz:=true
```

双雷达 safety 模式。这个命令会启动一个 Livox driver，并按 IP 分流两个 MID-360：

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
  enable_second_lidar_safety:=true \
  use_rviz:=true
```

双雷达默认使用：

- `dual_lidar_user_config_path:=<rm_bringup>/config/pb2025_xmu_dual_mid360_config.json`
- `front_lidar_custom_topic:=livox/lidar_192_168_1_173`
- `front_imu_custom_topic:=livox/imu_192_168_1_173`
- `second_lidar_custom_topic:=/livox/lidar_192_168_1_166`
- `second_lidar_pointcloud_topic:=/second_livox/lidar`

默认主雷达 IP 是 `192.168.1.173`，第二雷达 IP 是 `192.168.1.166`，本机有线网卡 IP 是 `192.168.1.2`。如果现场改了雷达 IP，改 `pb2025_xmu_dual_mid360_config.json` 和对应 topic 参数；单雷达稳定配置 `pb2025_xmu_mid360_config.json` 不要动。

`second_livox_only.launch.py` 只保留作单独硬件排查，不要和 PB 主链同时启动。

两只 MID-360 都上电且在同一交换机时，建议直接跑双雷达 safety 模式。单雷达模式仍保持 `/livox/lidar` 和 `/livox/imu` 语义，但 Livox SDK 可能会打印第二雷达未在单雷达配置中声明的提示。

## tmux 一键启动

完整整车 + 第二雷达 safety 可以直接使用：

```bash
cd /home/rm/Desktop/SENTRY_FULL/XMU_RCS_SENTRY
source /opt/ros/humble/setup.bash
source install/setup.bash
./tools/start_dual_lidar_full_tmux.sh
```

脚本会创建 `sentry_dual` session：

- `full_bringup`：启动完整整车，一个 Livox driver 同时接两个 MID-360，并设置 `enable_second_lidar_safety:=true`
- `monitor`：保留常用检查命令

进入和停止：

```bash
tmux attach -t sentry_dual
tmux kill-session -t sentry_dual
```

常用覆盖：

```bash
USE_RVIZ=true ./tools/start_dual_lidar_full_tmux.sh
SERIAL_DEVICE=/dev/rm_serial ./tools/start_dual_lidar_full_tmux.sh
MAP=/home/rm/Desktop/SENTRY_FULL/maps/self_filtered_map.yaml ./tools/start_dual_lidar_full_tmux.sh
BRINGUP_ARGS='log_level:=debug' ./tools/start_dual_lidar_full_tmux.sh
```

## 检查

```bash
ros2 run tf2_ros tf2_echo base_link second_mid360
ros2 topic hz /livox/lidar_192_168_1_173
ros2 topic hz /livox/imu_192_168_1_173
ros2 topic hz /livox/lidar_192_168_1_166
ros2 topic hz /second_livox/lidar
ros2 topic hz /second_lidar_obstacle_cloud
ros2 topic hz /cmd_vel
ros2 topic hz /cmd_vel_safe
ros2 topic echo /second_lidar_obstacle_debug
ros2 topic echo /second_lidar_safety_debug
```

RViz 可直接显示的点云 topic：

```text
/cloud_registered
/terrain_map
/second_livox/lidar
/second_lidar_obstacle_cloud
```

`/livox/lidar` 和 `/livox/lidar_192_168_1_xxx` 是 CustomMsg，不要在 RViz 里作为普通 PointCloud2 显示源。

验收重点：

- 单雷达模式 `/livox/lidar` 和 `/livox/imu` 不变。
- 双雷达模式 Point-LIO 只使用 `/livox/lidar_192_168_1_173` 和 `/livox/imu_192_168_1_173`。
- 第二雷达不进入 costmap、terrain、建图、定位、small_gicp。
- `enable_second_lidar_safety:=false` 时没有任何 `second_lidar` 节点。
