# XMU RoboMaster 哨兵 2026 赛季 ROS 2 工作区

当前仓库包含三条底层能力链：

- **视觉自瞄链**：`rm_hik_driver → rm_vision → rm_autoaim → rm_hw_bridge`
- **定位链**：`Mid360 → Point-LIO → small_gicp → map→odom→base_link`
- **导航链**：`Nav2(NavFn+DWB) → /cmd_vel → /nav_cmd → rm_hw_bridge → 底盘`

---

## 1. 工作区结构

| 包 | 作用 |
|---|---|
| `rm_interfaces` | 自定义消息：`GimbalStatus`、`GimbalCmd`、`NavCmd`、`LocalizationStatus` 等 |
| `rm_hw_bridge` | 串口桥，上下位机协议翻译；`game_progress`、`robot_id` 从这里上来 |
| `rm_hik_driver` | 海康相机驱动 |
| `rm_vision` | YOLO 装甲板检测；支持裁判 `robot_id` 自动决定敌我颜色 |
| `rm_autoaim` | PnP、Tracker、Aimer、自瞄命令发布 |
| `rm_livox_driver` | Mid360 驱动 bringup |
| `rm_point_lio` | Point-LIO wrapper（参数覆盖） |
| `point_lio` | Point-LIO 本体 |
| `rm_global_localization` | 基于 `small_gicp` 的全局重定位，输出 `map → odom` TF |
| `small_gicp` | header-only 配准库 |
| `rm_bringup` | 全仓启动编排、Nav2 参数、`cmd_vel` 桥接、`initial_pose` 管理、障碍点云过滤 |

---

## 2. 从零编译

### 2.1 环境

- Ubuntu 22.04
- ROS 2 Humble
- OpenVINO 2024
- PCL / Eigen / OpenCV
- 海康 MVS SDK

### 2.2 编译

```bash
cd ~/Desktop/SENTRY_FULL/XMU_RCS_SENTRY
source /opt/ros/humble/setup.bash

# 全编
colcon build --symlink-install
source install/setup.bash
```

如果搬过目录，先清旧产物：

```bash
rm -rf build/ install/ log/
colcon build --symlink-install
```

只编导航相关（不含自瞄/视觉）：

```bash
colcon build --symlink-install \
  --packages-skip rm_vision rm_autoaim rm_hik_driver
```

---

## 3. 地图文件位置

### 3.1 PCD 地图（给 small_gicp 定位用）

| 文件 | 路径 | 说明 |
|------|------|------|
| 赛事先验地图 | `~/Desktop/SENTRY_FULL/RMUC2026.pcd` | 官方/先验 PCD |
| 实验室建图 | `~/Desktop/SENTRY_FULL/scans.pcd` | Point-LIO 最新建图结果 |

### 3.2 二维栅格地图（给 Nav2 map_server 用）

所有二维地图统一放在 `~/Desktop/SENTRY_FULL/lab_maps/` 下：

| 文件 | 说明 |
|------|------|
| `lab_maps/test_v2.yaml` + `test_v2.pgm` | **当前推荐使用的导航地图** |
| `lab_maps/test_v2_obstacle_mask.pgm` | 障碍物掩膜 |
| `lab_maps/test_v2_ground_mask.pgm` | 地面掩膜 |
| `lab_maps/test_v2_known_mask.pgm` | 已知区域掩膜 |
| `lab_maps/nav_map.yaml` + `nav_map.pgm` | 另一版导航地图 |
| `lab_maps/test.yaml` + `test.pgm` | 早期测试地图 |

### 3.3 一句话总结

- **PCD** → `small_gicp` / `rm_global_localization` 做定位配准
- **PGM + YAML** → Nav2 `map_server` 做全局规划
- 两者必须基于同一次建图，坐标系一致

---

## 4. 启动模式

所有启动都通过一个 launch 文件：`sentry_bringup.launch.py`

### 4.1 模式一：完整整车（自瞄 + 导航 + 串口）

```bash
source install/setup.bash

ros2 launch rm_bringup sentry_bringup.launch.py \
  use_serial:=true \
  enable_navigation:=true \
  enable_global_localization:=true \
  enable_nav2:=true \
  global_map_path:=/home/rm/Desktop/SENTRY_FULL/scans.pcd \
  nav2_map_yaml:=/home/rm/Desktop/SENTRY_FULL/lab_maps/test_v2.yaml \
  serial_device:=/dev/ttyUSB0
```

启动内容：
- `rm_hw_bridge`（串口）
- `rm_hik_driver`（相机，延迟 2s）
- `rm_vision` + `rm_autoaim`（自瞄，延迟 4s）
- Livox + Point-LIO（定位链，延迟 0.5s + 1.5s）
- `rm_global_localization`（重定位，延迟 3s）
- Nav2 全栈（延迟 4.5s）

### 4.2 模式二：纯导航（含串口，不含自瞄）

**日常调导航用这个。**

```bash
source install/setup.bash

ros2 launch rm_bringup sentry_bringup.launch.py \
  navigation_only:=true \
  use_serial:=true \
  enable_global_localization:=true \
  enable_nav2:=true \
  global_map_path:=/home/rm/Desktop/SENTRY_FULL/scans.pcd \
  nav2_map_yaml:=/home/rm/Desktop/SENTRY_FULL/lab_maps/test_v2.yaml
```

启动内容：
- `rm_hw_bridge`（串口）
- Livox + Point-LIO + small_gicp + Nav2
- **不启动**相机/自瞄

### 4.3 模式三：纯自瞄（无导航）

```bash
source install/setup.bash

ros2 launch rm_bringup sentry_bringup.launch.py \
  use_serial:=true \
  serial_device:=/dev/ttyUSB0
```

启动内容：
- `rm_hw_bridge`（串口）
- 相机 + 自瞄栈
- **不启动**导航

### 4.4 模式四：纯定位链（调定位/建图）

```bash
source install/setup.bash

ros2 launch rm_bringup sentry_bringup.launch.py \
  navigation_only:=true \
  use_serial:=false
```

只启动 Livox + Point-LIO，用于建图或调试定位。

### 4.5 模式五：纯 Nav2（定位链已单独运行时）

```bash
source install/setup.bash

ros2 launch rm_bringup pure_navigation_bringup.launch.py \
  map:=/home/rm/Desktop/SENTRY_FULL/lab_maps/test_v2.yaml \
  params_file:=/home/rm/Desktop/SENTRY_FULL/XMU_RCS_SENTRY/rm_bringup/config/sentry_nav2_params.yaml
```

---

## 5. Launch 参数速查

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `use_serial` | `true` | 启动串口 (`rm_hw_bridge`) |
| `serial_device` | `/dev/ttyUSB0` | 串口设备路径 |
| `baudrate` | `460800` | 波特率 |
| `navigation_only` | `false` | 仅启动导航链（不含相机/自瞄） |
| `enable_navigation` | `false` | 启动导航链（含相机/自瞄） |
| `enable_global_localization` | `false` | 启动 small_gicp 重定位 |
| `enable_nav2` | `false` | 启动 Nav2 导航栈 |
| `global_map_path` | `.../RMUC2026.pcd` | PCD 地图路径 |
| `nav2_map_yaml` | `""` | 二维地图 YAML 路径 |
| `enable_decision` | `false` | 启动 sentry_bt 决策 |
| `use_sim_time` | `false` | 使用仿真时钟 |
| `nav2_rviz` | `true` | Nav2 启动时开 RViz |
| `color_ignore` | `1` | 忽略颜色：0=红, 1=蓝, -1=无 |
| `target_color` | `red` | 目标颜色：red/blue/all |
| `publish_debug_image` | `false` | 发布调试图像 |
| `model_path` | `model/yolo11.xml` | OpenVINO 模型路径 |

---

## 6. 建图：Mid360 → Point-LIO → PCD

### 6.1 开启建图模式

修改 `rm_point_lio/config/mid360_point_lio.yaml`：

```yaml
pcd_save:
  pcd_save_en: true
  interval: -1
```

### 6.2 启动并建图

```bash
ros2 launch rm_bringup sentry_bringup.launch.py \
  navigation_only:=true use_serial:=false
```

- 静止 5~10s 让 IMU 初始化
- 缓慢移动覆盖场地
- 尽量回到起点做回环

### 6.3 保存

`Ctrl+C` 退出后 PCD 落到 `point_lio/PCD/scans.pcd`，立即备份：

```bash
cp point_lio/PCD/scans.pcd ~/Desktop/SENTRY_FULL/scans.pcd
```

### 6.4 PCD → PGM/YAML

```bash
python3 rm_bringup/scripts/pcd2pgm.py \
  ~/Desktop/SENTRY_FULL/scans.pcd \
  --output-dir ~/Desktop/SENTRY_FULL/lab_maps \
  --map-name my_map \
  --resolution 0.05 \
  --z-min 0.15 \
  --z-max 1.80 \
  --ground-band-min -0.20 \
  --ground-band-max 0.10 \
  --radius-outlier-nb-points 2 \
  --radius-outlier-radius 0.15 \
  --closing-kernel 3 \
  --known-dilate-kernel 3
```

---

## 7. 裁判信息、敌我识别、initialpose

### 7.1 裁判信息

下位机通过 `rm_hw_bridge` 上行：
- `/gimbal_status.game_progress`：`0`=未开始, `1`=准备, `2`=自检, `3`=倒计时, `4`=比赛中, `5`=结算
- `/gimbal_status.robot_id`：红方 `1~11`，蓝方 `101~111`

### 7.2 敌我识别

`rm_vision` 优先级：
1. 用 `robot_id` 推导己方颜色 → 自动反推敌方
2. 无裁判信息时回退 YAML 的 `target_color` / `color_ignore`

### 7.3 初始位姿

`rm_initial_pose_manager` 负责：
1. 启动后自动发一轮 `/initialpose`
2. 收到 `game_progress == 3` 时按 `robot_id` 重发

配置在 `rm_bringup/config/initial_pose_manager.yaml`：

```yaml
red_start_pose:  [x, y, z, yaw_deg]   # ← 按先验地图坐标填写
blue_start_pose: [x, y, z, yaw_deg]   # ← 按先验地图坐标填写
fallback_robot_id: 7
```

---

## 8. 标定

### 8.1 相机内参

填写到 `rm_autoaim/config/params.yaml`：

```yaml
camera_matrix: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
dist_coeffs: [k1, k2, p1, p2, k3]
```

### 8.2 相机到云台外参

同样在 `rm_autoaim/config/params.yaml`：

```yaml
r_cam_to_gimbal: [...]
t_cam_to_gimbal: [...]
```

### 8.3 雷达到车体外参

`base_link → livox_frame` 静态 TF 在 `sentry_bringup.launch.py` 的 launch 参数中：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `lidar_x` | `0.0` | 前向偏移 (m) |
| `lidar_y` | `0.2` | 左向偏移 (m) |
| `lidar_z` | `0.35` | 高度 (m) |
| `lidar_roll` | `0.0` | 横滚 (rad) |
| `lidar_pitch` | `0.3115` | 俯仰 (rad)，后仰 ~17.85° |
| `lidar_yaw` | `1.5708` | 偏航 (rad)，左转 90° |

**雷达外参变了，就必须重新验证定位，自建地图通常还要重建。**

---

## 9. 导航架构说明

### 9.1 Nav2 配置

| 组件 | 配置 |
|------|------|
| Planner | `NavFnPlanner`（A*） |
| Controller | `DWBLocalPlanner`（全向：vx/vy/wz） |
| Global Costmap | `static_layer` + `inflation_layer` |
| Local Costmap | `obstacle_layer`（吃 `/nav_obstacle_cloud`）+ `inflation_layer` |
| BT | `navigate_to_pose_w_replanning_and_recovery.xml` |
| AMCL | **不启用**（由 `rm_global_localization` 提供定位） |

参数文件：`rm_bringup/config/sentry_nav2_params.yaml`

### 9.2 速度输出链

```
Nav2 controller_server → /cmd_vel (Twist)
  → cmd_vel_to_nav_cmd.py → /nav_cmd (NavCmd)
    → rm_hw_bridge → 串口 → 底盘 NAVI
```

### 9.3 障碍物感知链

```
Point-LIO → /cloud_registered (PointCloud2)
  → obstacle_cloud_filter.py → /nav_obstacle_cloud (过滤后障碍点云)
    → Nav2 local_costmap obstacle_layer
```

`obstacle_cloud_filter.py` 的作用：
- 从 Point-LIO 注册点云中提取障碍物
- 高度裁剪（去掉地面和天花板）
- 转换到 `odom` 坐标系
- 输出给 Nav2 local costmap

---

## 10. 职责边界（不要打乱）

| 模块 | 职责 | 不做 |
|------|------|------|
| `rm_hw_bridge` | 协议桥 | 不加业务逻辑 |
| `rm_vision` | 检测+颜色过滤 | 不做追踪/瞄准 |
| `rm_autoaim` | PnP/Tracker/Aimer | 不做检测 |
| `point_lio` | 局部 odom (`odom→base_link`) | 不管全局 |
| `rm_global_localization` | 全局定位 (`map→odom`) | 不管局部 odom |
| `rm_bringup` | 启动编排+Nav2+桥接 | 不做定位/自瞄 |
| `obstacle_cloud_filter` | 点云过滤给 costmap | 不做配准/定位 |

---

## 11. 常用调试命令

```bash
# 查 Nav2 各节点 lifecycle 状态
ros2 lifecycle list /bt_navigator
ros2 lifecycle get /controller_server

# 查速度输出
ros2 topic echo /cmd_vel
ros2 topic hz /nav_cmd

# 查定位状态
ros2 topic echo /localization_status

# 查 TF 链
ros2 run tf2_ros tf2_echo map base_link

# 查裁判状态
ros2 topic echo /gimbal_status

# 查障碍点云
ros2 topic hz /nav_obstacle_cloud

# 查 initialpose
ros2 topic echo /initialpose
```

---

## 12. 上车检查清单

### 雷达标定前

- [ ] `base_link → livox_frame` 外参是否最新（launch 参数）
- [ ] `red_start_pose / blue_start_pose` 是否按先验地图坐标填写
- [ ] `fallback_robot_id` 是否正确

### 定位前

- [ ] `global_map_path` 指向正确 PCD
- [ ] `/cloud_registered` 正常发布
- [ ] `/initialpose` 是否已自动发出
- [ ] `map → odom → base_link` TF 链完整

### Nav2 前

- [ ] 已有 `pgm + yaml` 二维地图
- [ ] `map → odom → base_link` 稳定
- [ ] `/nav_obstacle_cloud` 正常发布
- [ ] 串口设备路径正确（`ls /dev/ttyUSB*`）

### 自瞄前

- [ ] 相机内参已标定
- [ ] 相机到云台外参已标定
- [ ] 模型文件路径正确
- [ ] `color_ignore` / `target_color` 配置正确（或依赖裁判系统自动判断）