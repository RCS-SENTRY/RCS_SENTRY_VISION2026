# XMU_RCS_SENTRY

XMU RoboMaster 哨兵 2026 赛季 ROS 2 工作区。  
当前仓库包含三条底层能力链：

- 视觉自瞄链：`rm_hik_driver -> rm_vision -> rm_autoaim -> rm_hw_bridge`
- 定位链：`Mid360 -> Point-LIO -> small_gicp -> map->odom->base_link`
- 纯底层导航链：`small_gicp TF + /cloud_registered + Nav2 -> /cmd_vel -> /nav_cmd`

这份 README 按“明天能上车”的顺序来写，只保留真正落地要用的流程。

---

## 1. 工作区结构

| 包 | 作用 |
|---|---|
| `rm_interfaces` | 自定义消息：`GimbalStatus`、`GimbalCmd`、`NavCmd`、`LocalizationStatus` 等 |
| `rm_hw_bridge` | 串口桥，负责上下位机协议翻译；`game_progress`、`robot_id` 从这里上来 |
| `rm_hik_driver` | 海康相机驱动 |
| `rm_vision` | YOLO 装甲板检测；现在支持“裁判 `robot_id` 优先决定敌我颜色” |
| `rm_autoaim` | PnP、Tracker、Aimer、自瞄命令发布 |
| `rm_livox_driver` | Mid360 驱动 bringup |
| `point_lio` / `rm_point_lio` | Point-LIO 本体与 wrapper |
| `rm_global_localization` | 基于 `small_gicp` 的全局重定位，输出 `map -> odom` |
| `rm_bringup` | 全仓启动编排、Nav2 参数、`cmd_vel` 桥接、`initial_pose` 管理 |
| `small_gicp` | header-only 配准库 |

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

# 第一次建议全编
colcon build --symlink-install

source install/setup.bash
```

如果之前目录搬过位置，建议先清掉旧产物：

```bash
rm -rf build/ install/ log/
colcon build --symlink-install
```

---

## 3. 裁判信息、敌我识别、`initialpose`

### 3.1 裁判信息从哪里来

下位机通过 `rm_hw_bridge` 上行到：

- `/gimbal_status.game_progress`
- `/gimbal_status.robot_id`

当前协议约定：

- `game_progress`
  - `0` 未开始比赛
  - `1` 准备阶段
  - `2` 15 秒裁判系统自检
  - `3` 5 秒倒计时
  - `4` 比赛中
  - `5` 比赛结算

- `robot_id`
  - 红方：`1 ~ 11`
  - 蓝方：`101 ~ 111`

### 3.2 敌我识别现在怎么做

`rm_vision` 现在的优先级是：

1. 优先使用 `/gimbal_status.robot_id` 推导己方颜色
2. 自动反推敌方目标颜色
3. 如果裁判信息无效或没有上来，再回退到 YAML：
   - `target_color`
   - `color_ignore`

所以：

- 红方机器人：自动打蓝色、忽略红色
- 蓝方机器人：自动打红色、忽略蓝色
- 纯调试没裁判信息：走 `rm_vision/config/params.yaml`

### 3.3 `initialpose` 现在谁来发

现在由 `rm_bringup` 里的 `rm_initial_pose_manager` 节点负责。

逻辑是：

1. 启动导航/定位链后，默认会发一轮 startup `initialpose`
2. 如果后续收到 `game_progress == 3`，会再按当前 `robot_id` 重发一轮
3. 比赛时这轮倒计时触发是第一优先级
4. 平时测试没有裁判信息时，startup 发布保证能直接调试

起始位姿参数在：

- [initial_pose_manager.yaml](./rm_bringup/config/initial_pose_manager.yaml)

需要填的关键项：

```yaml
red_start_pose:  [x, y, z, yaw_deg]
blue_start_pose: [x, y, z, yaw_deg]
fallback_robot_id: 7
```

---

## 4. 标定

### 4.1 相机内参 + 畸变

填写到：

- [rm_autoaim/config/params.yaml](./rm_autoaim/config/params.yaml)

关键字段：

```yaml
camera_matrix: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
dist_coeffs: [k1, k2, p1, p2, k3]
```

### 4.2 相机到云台外参

同样在：

- [rm_autoaim/config/params.yaml](./rm_autoaim/config/params.yaml)

关键字段：

```yaml
r_cam_to_gimbal: [...]
t_cam_to_gimbal: [...]
```

### 4.3 雷达到车体外参

当前 `base_link -> livox_frame` 静态 TF 是在：

- [sentry_bringup.launch.py](./rm_bringup/launch/sentry_bringup.launch.py)

统一发布的。

注意：

- 现在 launch 里默认只显式参数化了 `z`
- 如果明天雷达重新标定后存在 `x/y/roll/pitch/yaw` 偏角，必须同步修改这里
- **先验地图、Point-LIO、small_gicp、Nav2 的坐标一致性都依赖这条 TF**

一句话：

**雷达外参变了，就必须重新验证定位；如果是自建地图，通常还要重建。**

---

## 5. 建图：Mid360 -> Point-LIO -> PCD

### 5.1 开启建图模式

Point-LIO 的建图保存开关在：

- [mid360_point_lio.yaml](./rm_point_lio/config/mid360_point_lio.yaml)

建图前确认：

```yaml
pcd_save:
  pcd_save_en: true
  interval: -1
```

### 5.2 启动纯导航链（只起雷达和 Point-LIO）

```bash
cd ~/Desktop/SENTRY_FULL/XMU_RCS_SENTRY
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch rm_bringup sentry_bringup.launch.py navigation_only:=true
```

### 5.3 建图动作

- 上电后静止 `5~10s`
- 再缓慢移动雷达或车体
- 尽量覆盖完整实验室 / 场地
- 尽量回到起点做回环

### 5.4 退出并保存

正常 `Ctrl+C` 退出后，PCD 会落到：

- `point_lio/PCD/scans.pcd`

建议立刻复制成固定名字：

```bash
cp point_lio/PCD/scans.pcd ~/Desktop/SENTRY_FULL/RMUC2026_lab_test.pcd
```

---

## 6. PCD -> PGM / YAML：给 Nav2 用的二维地图

### 6.1 先分清楚

- `PCD`：给 `small_gicp` 做定位
- `PGM/PNG + YAML`：给 Nav2 `map_server` 做二维全局规划

**只有 PCD，不够跑当前这版完整 Nav2。**

### 6.2 推荐链路

明天上车如果需要完整 Nav2，请按这条线：

1. 用 Point-LIO 录出 `PCD`
2. 在 CloudCompare 打开 PCD
3. 做高度裁剪 / 去除无关高层结构
4. 顶视图正交投影
5. 导出为二维黑白图（`png/pgm`）
6. 在图像软件里清理成占据栅格
7. 写 `map.yaml`

### 6.3 `map.yaml` 示例

```yaml
image: RMUC2026_lab_test.pgm
mode: trinary
resolution: 0.05
origin: [0.0, 0.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

### 6.4 一句话总结

明天完整链路是：

`Mid360 -> Point-LIO -> PCD -> 2D栅格图(pgm/yaml) -> Nav2`

---

## 7. 全局重定位：small_gicp

### 7.1 启动链

```bash
cd ~/Desktop/SENTRY_FULL/XMU_RCS_SENTRY
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch rm_bringup sentry_bringup.launch.py \
  navigation_only:=true \
  enable_global_localization:=true \
  global_map_path:=/home/rm/Desktop/SENTRY_FULL/RMUC2026_lab_test.pcd
```

### 7.2 当前输出

- `map -> odom -> base_link`
- `/global_map`
- `/localized_pose`
- `/localization_status`

### 7.3 `/initialpose` 的作用

它是给 `small_gicp` 一个**局部收敛初值**，不是给底盘直接开车的。

当前这套不是“完全无先验全局定位”，而是：

- 有先验地图
- 有一个大致初始位姿
- 然后做 scan-to-map 局部收敛

---

## 8. 纯底层 Nav2

### 8.1 现在这版 Nav2 包含什么

- 全局规划器：`SmacPlanner2D`
- 局部控制器：`MPPIController`
- 全向底盘配置：允许 `vx / vy / wz`
- `local_costmap` 直接吃 `/cloud_registered` 的 3D 点云，压成 2D 障碍
- 不启用 `amcl`

参数文件：

- [sentry_nav2_params.yaml](./rm_bringup/config/sentry_nav2_params.yaml)

启动文件：

- [pure_navigation_bringup.launch.py](./rm_bringup/launch/pure_navigation_bringup.launch.py)

### 8.2 启动 Nav2

前提：

1. 定位链已经输出稳定的 `map -> odom -> base_link`
2. 你已经有一张二维 `map.yaml`

```bash
cd ~/Desktop/SENTRY_FULL/XMU_RCS_SENTRY
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch rm_bringup pure_navigation_bringup.launch.py \
  map:=/absolute/path/to/your_map.yaml
```

### 8.3 输出到底盘

当前 Nav2 输出链：

`/cmd_vel -> cmd_vel_to_nav_cmd.py -> /nav_cmd -> rm_hw_bridge -> 下位机 NAVI`

这条链只做纯底层速度桥接，不带战术逻辑。

---

## 9. 完整测试链路

### 9.1 视觉自瞄

```bash
ros2 launch rm_bringup sentry_bringup.launch.py
```

### 9.2 雷达建图

```bash
ros2 launch rm_bringup sentry_bringup.launch.py navigation_only:=true
```

### 9.3 定位验收

```bash
ros2 launch rm_bringup sentry_bringup.launch.py \
  navigation_only:=true \
  enable_global_localization:=true \
  global_map_path:=/home/rm/Desktop/SENTRY_FULL/RMUC2026_lab_test.pcd
```

### 9.4 底层 Nav2

先起定位链，再起：

```bash
ros2 launch rm_bringup pure_navigation_bringup.launch.py \
  map:=/absolute/path/to/map.yaml
```

---

## 10. 明天上车最重要的检查项

### 雷达标定前

- [ ] `base_link -> livox_frame` 外参是否最新
- [ ] `red_start_pose / blue_start_pose` 是否按先验地图填写
- [ ] `fallback_robot_id` 是否正确

### 定位前

- [ ] `global_map_path` 指向正确 PCD
- [ ] `/global_map`、`/localized_pose`、`/localization_status` 正常
- [ ] `/initialpose` 是否已自动发出

### Nav2 前

- [ ] 已经有 `pgm/png + yaml`
- [ ] `map -> odom -> base_link` 稳定
- [ ] `/cloud_registered` 正常

---

## 11. 常用命令

### 看裁判状态

```bash
ros2 topic echo /gimbal_status
```

### 看定位状态

```bash
ros2 topic echo /localization_status
```

### 看 `initialpose` 是否被发布

```bash
ros2 topic echo /initialpose
```

### 看 Nav2 输出

```bash
ros2 topic hz /cmd_vel
ros2 topic hz /nav_cmd
```

---

## 12. 当前工程约定

- `rm_hw_bridge`：只做协议桥，不加业务逻辑
- `rm_vision`：负责检测与敌我颜色过滤
- `rm_autoaim`：负责 PnP / Tracker / Aimer / `/gimbal_cmd`
- `point_lio`：只管局部 odom
- `rm_global_localization`：只管 `map -> odom`
- `rm_bringup`：做启动编排、`initialpose` 管理、Nav2 纯底层桥接

这条边界后面尽量不要再打乱。
