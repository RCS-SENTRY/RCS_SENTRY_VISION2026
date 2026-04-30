# RCS Sentry Vision 2026

RoboMaster 哨兵上位机 ROS 2 工作区，当前主线为：

- XMU 通信栈：`rm_hw_bridge` + `rm_interfaces`
- XMU 自瞄栈：`rm_hik_driver` + `rm_vision` + `rm_autoaim`
- PB2025 navigation stack
- XMU PB-to-NavCmd bridge：`Twist` -> `/nav_cmd`

旧 XMU 自研导航栈已删除。需要追溯时使用 git history。

## 当前架构

```text
rm_hik_driver -> rm_vision -> rm_autoaim -> rm_hw_bridge -> lower MCU

PB2025:
  livox_ros_driver2 -> point_lio -> loam_interface
  -> sensor_scan_generation -> terrain_analysis / terrain_analysis_ext
  -> Nav2 + pb_omni_pid_pursuit_controller
  -> velocity_smoother -> fake_vel_transform -> /cmd_vel
  -> pb_cmd_vel_to_nav_cmd.py -> /nav_cmd -> rm_hw_bridge

PB2025 + second lidar safety limiter (recommended dual lidar mode):
  /cmd_vel -> second_lidar_safety_limiter -> /cmd_vel_safe
  -> pb_cmd_vel_to_nav_cmd.py -> /nav_cmd -> rm_hw_bridge
```

`rm_hw_bridge` 的串口协议、`rm_interfaces/msg/NavCmd` 格式和 `/nav_cmd` 订阅接口保持不变。

## 第三方声明

- PB2025 来源：<https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav>
- License：Apache-2.0
- 上游 `LICENSE` 与 `README.md` 保留在 `src/third_party/pb2025_sentry_nav/`
- XMU 修改说明见 `src/third_party/pb2025_sentry_nav/XMU_ADAPTATION.md`

## 目录结构

```text
src/rm_interfaces/                 XMU 自定义消息，包含 NavCmd
src/rm_hw_bridge/                  XMU 串口通信桥
src/rm_hik_driver/                 海康相机驱动
src/rm_vision/                     装甲板识别
src/rm_autoaim/                    自瞄解算与云台命令
src/rm_bringup/                    一键启动、PB2025 XMU 参数、PB-to-NavCmd bridge
src/sentry_bt/                     可选决策节点
src/third_party/pb2025_sentry_nav/ PB2025 导航源码
model/                         视觉模型文件
```

## 编译依赖

- Ubuntu 22.04
- ROS 2 Humble
- Nav2 / slam_toolbox / tf2 / PCL / Eigen
- OpenVINO 与海康 MVS SDK
- Livox SDK2 运行环境

安装依赖并编译：

```bash
cd /home/rm/Desktop/SENTRY_FULL/XMU_RCS_SENTRY
source /opt/ros/humble/setup.bash
rosdep install -r --from-paths src --ignore-src --rosdistro humble -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

本仓库采用标准 `src/` colcon 工作区布局，PB2025 源码位于 `src/third_party/pb2025_sentry_nav/`，`rosdep` 和 `colcon` 都从同一棵源码树发现包。

## 启动命令

完整整车：

```bash
ros2 launch rm_bringup sentry_bringup.launch.py \
  use_serial:=true \
  serial_device:=/dev/rm_serial \
  baudrate:=460800 \
  enable_navigation:=true \
  navigation_only:=false \
  slam:=False \
  map:=/home/rm/Desktop/SENTRY_FULL/maps/self_filtered_map.yaml \
  enable_small_gicp:=false \
  use_rviz:=false
```

仅导航：

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
  use_rviz:=false
```

仅自瞄：

```bash
ros2 launch rm_bringup sentry_bringup.launch.py \
  vision_only:=true \
  use_serial:=true \
  serial_device:=/dev/rm_serial \
  baudrate:=460800
```

无串口调试：

```bash
ros2 launch rm_bringup sentry_bringup.launch.py \
  debug_no_serial:=true \
  enable_navigation:=true \
  slam:=False \
  map:=/home/rm/Desktop/SENTRY_FULL/maps/self_filtered_map.yaml \
  use_rviz:=false
```

## 关键参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `use_serial` | `true` | 启动 `rm_hw_bridge` |
| `serial_device` | `/dev/rm_serial` | 串口设备，长期使用 udev 固定别名或 `/dev/serial/by-id/...` |
| `baudrate` | `460800` | 串口波特率 |
| `navigation_only` | `false` | 通信 + PB2025 导航，不启动相机/自瞄 |
| `vision_only` | `false` | 通信 + 相机/视觉/自瞄，不启动导航 |
| `debug_no_serial` | `false` | 跳过串口，其他按模式启动 |
| `enable_navigation` | `true` | 启动 PB2025 导航 |
| `enable_small_gicp` | `false` | 默认关闭 PB small_gicp 重定位 |
| `use_rviz` | `false` | 默认机器人端不开 RViz |
| `slam` | `False` | `False` 使用已有地图，`True` 进入 PB SLAM 模式 |
| `map` | `/home/rm/Desktop/SENTRY_FULL/maps/self_filtered_map.yaml` | Nav2 栅格地图 |
| `prior_pcd_file` | `/home/rm/Desktop/SENTRY_FULL/maps/self_filtered_scans.pcd` | 可选 PCD 先验 |
| `max_linear_speed` | `2.0` | PB-to-NavCmd bridge 最终合速度限幅 |
| `max_linear_accel` | `2.5` | PB-to-NavCmd bridge 线加速度限幅 |
| `cmd_vel_timeout_sec` | `0.25` | `/cmd_vel` 超时后持续发布零速 `/nav_cmd` |
| `force_zero_angular_z` | `true` | 默认屏蔽 PB `/cmd_vel.angular.z` |
| `enable_heading_align` | `false` | 预留 heading align 能力，默认关闭 |
| `enable_second_lidar` | `false` | 第二 MID-360 总开关，默认关闭 |
| `enable_second_lidar_filter` | `true` | 第二雷达点云过滤器开关，需要配合 `enable_second_lidar:=true` |
| `enable_second_lidar_safety_limiter` | `true` | 第二雷达近场安全壳，启用后 `/cmd_vel` 先限速为 `/cmd_vel_safe` |
| `enable_second_lidar_costmap` | `false` | 仅用于 debug，允许 second lidar 写 local costmap，不推荐比赛默认使用 |

默认行为树不执行 `BackUp` / `Spin` / `DriveOnHeading` 这类移动 recovery。默认 `force_zero_angular_z:=true` 且 `enable_heading_align:=false`；比赛前若要打开角速度，必须单独测试 `angular_z` 的方向和幅值。到点成功时 bridge 会在 `goal_reached_latch_sec` 时间内发布零速且 `is_reached=1` 的 `/nav_cmd`。

XMU MID-360 外参默认值：

```text
lidar_x=0.0, lidar_y=0.2, lidar_z=0.35
lidar_roll=0.0, lidar_pitch=0.3115, lidar_yaw=1.5708
```

第二 MID-360 默认关于车体 x 轴镜像，默认不写入 local costmap，而是作为近场 safety limiter：

```text
second_lidar_x=0.0, second_lidar_y=-0.2, second_lidar_z=0.35
second_lidar_roll=0.0, second_lidar_pitch=0.3115, second_lidar_yaw=-1.5708
```

XMU 车体尺寸：

```text
length=0.50, width=0.50, height=0.55
```

## 地图与 PCD

- `map` 指向 Nav2 使用的 `.yaml` 栅格地图。
- `prior_pcd_file` 指向 PB 点云先验，只有启用对应功能时才使用。
- 默认 `enable_small_gicp:=false`，低负载实车验证先跑 PB odom、terrain map、Nav2 与 `/nav_cmd` 闭环。
- 当前仍是单雷达定位：`front_mid360` 参与 Point-LIO；第二雷达不参与 Point-LIO、不参与建图、不参与 small_gicp。
- 坡道若仍被识别为障碍，先录包判断是 `/terrain_map` 错，还是 costmap 解释错；可小步尝试 `quantileZ: 0.30`、`maxRelZ/upperBoundZ: 0.70`、`disRatioZ: 0.30`。

双雷达避障（safety limiter 模式，推荐）：

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
  enable_second_lidar:=true \
  enable_second_lidar_filter:=true \
  enable_second_lidar_safety_limiter:=true \
  enable_second_lidar_costmap:=false \
  use_rviz:=false

debug 才允许开启 second lidar costmap（不推荐比赛默认开启）：

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
  enable_second_lidar:=true \
  enable_second_lidar_filter:=true \
  enable_second_lidar_safety_limiter:=true \
  enable_second_lidar_costmap:=true \
  use_rviz:=true
```
```

启用第二雷达前，需要按实车填写 `src/rm_bringup/config/pb2025_xmu_second_mid360_config.json` 中的第二雷达 IP / broadcast code / host IP。若出现大云台旋转导致鬼障碍，先确认没有误开 `enable_second_lidar_costmap`，再根据 `second_lidar_safety_limiter` 的 debug 输出调小作用范围。

推荐启动命令：

单雷达稳定主链：

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
  enable_second_lidar:=false \
  use_rviz:=true
```

双雷达 safety limiter 模式：

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
  enable_second_lidar:=true \
  enable_second_lidar_filter:=true \
  enable_second_lidar_safety_limiter:=true \
  enable_second_lidar_costmap:=false \
  use_rviz:=true
```

## 常用调试

```bash
ros2 node list
ros2 topic hz /odometry
ros2 topic hz /terrain_map
ros2 topic hz /terrain_map_ext
ros2 topic hz /cmd_vel
ros2 topic hz /cmd_vel_safe
ros2 topic hz /nav_cmd
ros2 topic echo /cmd_vel
ros2 topic echo /cmd_vel_safe
ros2 topic echo /nav_cmd
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_link second_mid360
ros2 topic list | grep second
ros2 topic hz /second_livox/lidar
ros2 topic hz /second_lidar_obstacle_cloud
ros2 topic echo /second_lidar_obstacle_debug
ros2 topic echo /second_lidar_safety_debug
ros2 topic echo /local_costmap/costmap --once
ros2 topic echo /global_costmap/costmap --once
ros2 param get /local_costmap/local_costmap second_lidar_obstacle_layer.enabled
```

到点检查：

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

到点后应看到 `linear_x=0`、`linear_y=0`、`angular_z=0`、`is_reached=1`。

## 低负载调试建议

- 机器人端用 SSH + tmux。
- 默认 `use_rviz:=false`。
- 本地电脑加入同一局域网，设置相同 `ROS_DOMAIN_ID` 后打开 RViz。
- RViz 只显示必要 topic。
- 需要复盘时录 rosbag 离线分析。
- 不推荐 ToDesk + RViz + 点云全开。
- 机器人端 RViz 若出现 GLSL sampler/link error，优先关闭机器人端 RViz，改用本地电脑 RViz；这通常是远程桌面或显卡 OpenGL 驱动组合问题，不是导航链路本身。

更多细节见 `docs/debugging_low_bandwidth.md`，串口固定见 `docs/serial_device_binding.md`，双雷达安全壳说明见 `docs/dual_lidar_safety_limiter.md`，双雷达上车测试见 `docs/dual_lidar_test_guide.md`。
