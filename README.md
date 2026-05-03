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
```

`rm_hw_bridge` 的串口协议、`rm_interfaces/msg/NavCmd` 格式和 `/nav_cmd` 订阅接口保持不变。

默认模式仍是 PB2025 单雷达稳定主链：`/livox/lidar`、`/livox/imu`、Point-LIO、terrain、local/global costmap 均保持 PB2025 接管版原始语义。

## 哨兵决策意图层

RM2026 sentry behavior tree 已按意图层接入：`sentry_bt` 只发布
`/sentry/intent` 和 `/sentry_bt/debug`，不直接发布 `/gimbal_cmd` 或
`/nav_cmd`。`rm_sentry_decision` 中的 command mux 合成最终
`/gimbal_cmd`，goal executor 只向 Nav2 发送 `NavigateToPose` goal，并通过
`/sentry/nav_status` 给决策层反馈战术到点状态。预设脚本模式由
`sentry_mission_runner_node` 发布 `/sentry/intent`，与 `sentry_bt` 二选一。

详细边界、姿态编码、规则字段限制和调试命令见
[docs/sentry_decision_integration.md](docs/sentry_decision_integration.md)。

第二雷达目前只作为可选速度安全壳：

- 双雷达 safety 模式使用单个 `livox_ros_driver2` 进程，按雷达 IP 分流 CustomMsg topic。
- 主雷达分流 topic：`/livox/lidar_192_168_1_173`、`/livox/imu_192_168_1_173`，只供 Point-LIO 使用。
- 第二雷达分流 topic：`/livox/lidar_192_168_1_166`，转换为 `/second_livox/lidar` 后进入 safety filter。
- 第二雷达不进入 Point-LIO，不参与定位。
- 第二雷达不进入建图、slam_toolbox 或 small_gicp。
- 第二雷达不进入 local/global costmap，不添加 obstacle layer。
- 单雷达默认模式不改变 `/livox/lidar` 和 `/livox/imu`。
- 开启 `enable_second_lidar_safety:=true` 时，PB `/cmd_vel` 先经 safety limiter 输出 `/cmd_vel_safe`，再由 `pb_cmd_vel_to_nav_cmd.py` 转为 `/nav_cmd`。

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

所有命令默认先进入工作区并加载环境：

```bash
cd /home/rm/Desktop/SENTRY_FULL/XMU_RCS_SENTRY
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### 仅导航栈

使用已有 Nav2 栅格地图，只启动通信 + PB2025 导航，不启动相机/识别/自瞄：

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
  use_rviz:=false
```

仅导航但不开串口，适合桌面/低风险调参：

```bash
ros2 launch rm_bringup sentry_bringup.launch.py \
  navigation_only:=true \
  debug_no_serial:=true \
  enable_navigation:=true \
  slam:=False \
  map:=/home/rm/Desktop/SENTRY_FULL/maps/self_filtered_map.yaml \
  use_rviz:=true
```

双雷达 safety 模式使用同一个 Livox driver，不需要单独启动第二雷达 driver：

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
  use_rviz:=false
```

### 仅自瞄栈

只启动通信 + 相机 + 识别 + 自瞄，不启动导航：

```bash
ros2 launch rm_bringup sentry_bringup.launch.py \
  vision_only:=true \
  use_serial:=true \
  serial_device:=/dev/rm_serial \
  baudrate:=460800 \
  enable_vision:=true \
  debug_no_serial:=false
```

无串口自瞄调试：

```bash
ros2 launch rm_bringup sentry_bringup.launch.py \
  vision_only:=true \
  use_serial:=false \
  debug_no_serial:=true \
  enable_vision:=true \
  publish_debug_image:=true
```

### 导航 + 自瞄同时开

完整整车：通信、PB2025 导航、相机、识别、自瞄同时启动。上位机语义保持为：无目标 `mode=0,state_switch=1,fire_control=0`；有目标但不能开火 `mode=1,state_switch=2,fire_control=0`；允许开火才发布 `fire_control=1`。

```bash
ros2 launch rm_bringup sentry_bringup.launch.py \
  use_serial:=true \
  serial_device:=/dev/rm_serial \
  baudrate:=460800 \
  enable_navigation:=true \
  enable_vision:=true \
  navigation_only:=false \
  vision_only:=false \
  slam:=False \
  map:=/home/rm/Desktop/SENTRY_FULL/maps/self_filtered_map.yaml \
  enable_small_gicp:=false \
  enable_second_lidar_safety:=false \
  use_rviz:=false
```

完整整车 + 第二雷达 safety：

```bash
ros2 launch rm_bringup sentry_bringup.launch.py \
  use_serial:=true \
  serial_device:=/dev/rm_serial \
  baudrate:=460800 \
  enable_navigation:=true \
  enable_vision:=true \
  slam:=False \
  map:=/home/rm/Desktop/SENTRY_FULL/maps/self_filtered_map.yaml \
  enable_second_lidar_safety:=true \
  use_rviz:=false
```

### 建图

建图使用 PB SLAM 模式，`slam:=True` 时只启动建图链路，不启动 Nav2 planner/controller/costmap。该模式会启动 `slam_toolbox` 与 `map_saver`，Point-LIO 也会在退出时保存 PCD。`use_rviz:=true` 时使用 PB 默认 RViz 视图，保持和导航调试一致的显示风格。建议建图时先不开自瞄，必要时不开串口。

```bash
ros2 launch rm_bringup sentry_bringup.launch.py \
  navigation_only:=true \
  debug_no_serial:=true \
  enable_navigation:=true \
  slam:=True \
  use_rviz:=true \
  enable_small_gicp:=false \
  enable_second_lidar_safety:=false
```

如果需要让底盘实际接收导航/遥控链路指令，使用串口建图：

```bash
ros2 launch rm_bringup sentry_bringup.launch.py \
  navigation_only:=true \
  use_serial:=true \
  serial_device:=/dev/rm_serial \
  baudrate:=460800 \
  enable_navigation:=true \
  slam:=True \
  use_rviz:=true \
  enable_small_gicp:=false \
  enable_second_lidar_safety:=false
```

保存 Nav2 栅格地图前，先确认 `/map` 已经开始发布。刚启动或车还没动起来时直接保存，`map_saver_cli` 可能会报 `Failed to spin map subscription`。

```bash
ros2 topic hz /map
```

确认 `/map` 有频率后保存：

```bash
mkdir -p /home/rm/Desktop/SENTRY_FULL/maps
ros2 run nav2_map_server map_saver_cli \
  -f /home/rm/Desktop/SENTRY_FULL/maps/new_map
```

建图结束后停止 launch，Point-LIO 会写出：

```text
src/third_party/pb2025_sentry_nav/point_lio/PCD/scans.pcd
```

需要作为先验点云时复制到地图目录：

```bash
cp src/third_party/pb2025_sentry_nav/point_lio/PCD/scans.pcd \
  /home/rm/Desktop/SENTRY_FULL/maps/new_scans.pcd
```

下次使用新地图导航：

```bash
ros2 launch rm_bringup sentry_bringup.launch.py \
  navigation_only:=true \
  use_serial:=true \
  serial_device:=/dev/rm_serial \
  baudrate:=460800 \
  enable_navigation:=true \
  slam:=False \
  map:=/home/rm/Desktop/SENTRY_FULL/maps/new_map.yaml \
  prior_pcd_file:=/home/rm/Desktop/SENTRY_FULL/maps/new_scans.pcd \
  enable_small_gicp:=false \
  use_rviz:=true
```

### 可视化调试

机器人端低负载运行时保持 `use_rviz:=false`。本地调试电脑加入同一局域网并设置同一个 `ROS_DOMAIN_ID` 后打开 RViz：

```bash
source /opt/ros/humble/setup.bash
source /home/rm/Desktop/SENTRY_FULL/XMU_RCS_SENTRY/install/setup.bash
ros2 run rviz2 rviz2
```

也可以直接让 launch 拉起 PB 默认 RViz：

```bash
ros2 launch rm_bringup sentry_bringup.launch.py \
  navigation_only:=true \
  debug_no_serial:=true \
  enable_navigation:=true \
  slam:=False \
  map:=/home/rm/Desktop/SENTRY_FULL/maps/self_filtered_map.yaml \
  use_rviz:=true
```

第二雷达默认配置：

- 主雷达 IP：`192.168.1.173`
- 第二雷达 IP：`192.168.1.166`
- 本机有线网卡 IP：`192.168.1.2`
- 主雷达 CustomMsg：`/livox/lidar_192_168_1_173`
- 主雷达 IMU：`/livox/imu_192_168_1_173`
- 第二雷达 CustomMsg：`/livox/lidar_192_168_1_166`
- 第二雷达 PointCloud2：`/second_livox/lidar`
- 双雷达配置文件：`src/rm_bringup/config/pb2025_xmu_dual_mid360_config.json`

如果现场 IP 变化，改双雷达配置文件和 launch topic 参数；单雷达稳定配置 `pb2025_xmu_mid360_config.json` 不要动。

两只 MID-360 都接在同一交换机且都上电时，推荐直接使用 `enable_second_lidar_safety:=true`。如果此时强行跑单雷达模式，Livox SDK 仍会发现第二雷达，日志里可能出现 `found lidar not defined in the user-defined config`，但主链 topic 仍保持单雷达语义。

也可以用 tmux 一键启动完整整车 + 第二雷达 safety：

```bash
./tools/start_dual_lidar_full_tmux.sh
```

常用覆盖参数：

```bash
USE_RVIZ=true ./tools/start_dual_lidar_full_tmux.sh
MAP=/home/rm/Desktop/SENTRY_FULL/maps/self_filtered_map.yaml ./tools/start_dual_lidar_full_tmux.sh
SERIAL_DEVICE=/dev/rm_serial ./tools/start_dual_lidar_full_tmux.sh
BRINGUP_ARGS='use_rviz:=true log_level:=debug' ./tools/start_dual_lidar_full_tmux.sh
```

进入或停止：

```bash
tmux attach -t sentry_dual
tmux kill-session -t sentry_dual
```

RViz 中显示点云时，选择 `sensor_msgs/msg/PointCloud2` topic，例如：

- `/cloud_registered`
- `/terrain_map`
- `/second_livox/lidar`
- `/second_lidar_obstacle_cloud`

不要把 `/livox/lidar`、`/livox/lidar_192_168_1_173` 或 `/livox/lidar_192_168_1_166` 当作 RViz 的普通点云显示源，它们是 `livox_ros_driver2/msg/CustomMsg`。

## 关键参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `use_serial` | `true` | 启动 `rm_hw_bridge` |
| `serial_device` | `/dev/rm_serial` | 串口设备，建议使用 udev 固定别名 |
| `baudrate` | `460800` | 串口波特率 |
| `navigation_only` | `false` | 通信 + PB2025 导航，不启动相机/自瞄 |
| `vision_only` | `false` | 通信 + 相机/视觉/自瞄，不启动导航 |
| `debug_no_serial` | `false` | 跳过串口，其他按模式启动 |
| `enable_navigation` | `true` | 启动 PB2025 导航 |
| `enable_small_gicp` | `false` | 默认关闭 PB small_gicp 重定位 |
| `enable_second_lidar_safety` | `false` | 可选第二雷达速度安全壳，不改变主导航链 |
| `use_rviz` | `false` | 默认机器人端不开 RViz |
| `slam` | `False` | `False` 使用已有地图，`True` 进入 PB SLAM 模式 |
| `map` | `/home/rm/Desktop/SENTRY_FULL/maps/self_filtered_map.yaml` | Nav2 栅格地图 |
| `prior_pcd_file` | `/home/rm/Desktop/SENTRY_FULL/maps/self_filtered_scans.pcd` | 可选 PCD 先验 |

XMU MID-360 外参默认值：

```text
lidar_x=0.0, lidar_y=0.2, lidar_z=0.35
lidar_roll=0.0, lidar_pitch=0.3115, lidar_yaw=1.5708
```

XMU 车体尺寸：

```text
length=0.50, width=0.50, height=0.55
```

## 地图与 PCD

- `map` 指向 Nav2 使用的 `.yaml` 栅格地图。
- `prior_pcd_file` 指向 PB 点云先验，只有启用对应功能时才使用。
- 默认 `enable_small_gicp:=false`，低负载实车验证先跑 PB odom、terrain map、Nav2 与 `/nav_cmd` 闭环。

## 常用调试

导航链路：

```bash
ros2 node list
ros2 topic hz /odometry
ros2 topic hz /terrain_map
ros2 topic hz /terrain_map_ext
ros2 topic hz /cmd_vel
ros2 topic hz /cmd_vel_safe
ros2 topic hz /nav_cmd
ros2 topic hz /livox/lidar_192_168_1_173
ros2 topic hz /livox/lidar_192_168_1_166
ros2 topic hz /second_livox/lidar
ros2 topic hz /second_lidar_obstacle_cloud
ros2 topic echo /second_lidar_obstacle_debug
ros2 topic echo /second_lidar_safety_debug
ros2 topic echo /cmd_vel
ros2 topic echo /nav_cmd
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 topic echo /local_costmap/costmap --once
ros2 topic echo /global_costmap/costmap --once
```

自瞄链路：

```bash
ros2 topic hz /detector/armors
ros2 topic hz /imu/data
ros2 topic echo /gimbal_status
ros2 topic echo /gimbal_cmd
ros2 topic echo /autoaim/debug_fire_gate
ros2 topic echo /autoaim/debug_world_points
```

完整整车联调时重点看：

```bash
ros2 topic echo /nav_cmd
ros2 topic echo /gimbal_cmd
ros2 topic echo /autoaim/debug_fire_gate
ros2 run tf2_ros tf2_echo map base_footprint
```

## 低负载调试建议

- 机器人端用 SSH + tmux。
- 默认 `use_rviz:=false`。
- 本地电脑加入同一局域网，设置相同 `ROS_DOMAIN_ID` 后打开 RViz。
- RViz 只显示必要 topic。
- 需要复盘时录 rosbag 离线分析。
- 不推荐 ToDesk + RViz + 点云全开。

更多细节见 `docs/debugging_low_bandwidth.md`。
