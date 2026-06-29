# RCS Sentry Vision 2026
特别致谢：
https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav
https://github.com/CSU-FYT-Vision/FYT2024_vision
# 哨兵上位机最小使用流程

这份说明给队内调车人员看。比赛或测试前，必须先完成两件事：

1. 先建图，保存当前场地的 `map.yaml`。
2. 把所有战术点坐标重新量好，写进 `src/rm_sentry_decision/config/sentry_goals.yaml`。

`sentry_goals.yaml` 里的默认坐标只是占位值，不能直接上场用。

## 0. 当前系统能做什么

当前主链：

```text
建图/定位/Nav2/PB2025 -> /cmd_vel -> /nav_cmd
自瞄 -> /autoaim/gimbal_cmd_raw
决策/脚本 -> /sentry/intent
command_mux -> /gimbal_cmd
goal_executor -> Nav2 goal + /sentry/nav_status
```

当前已经支持：

1. 根据 `goal_id` 自动发 Nav2 目标点。
2. 到目标附近后由 tactical reach 判定到点。
3. 到点后通过 `/sentry/nav_status.reached` 触发下一步。
4. `mission_runner` 按 YAML 脚本自动巡航。
5. `sentry_bt` 根据血量、弹量、热量、自瞄敌情、导航状态输出意图。
6. `command_mux` 合成最终 `/gimbal_cmd`。

当前还不保证：

1. 不建图就能跑。
2. `goal` 坐标不用标定。
3. 官方远程回血、远程补弹、复活一定生效。
4. 下位机一定已经执行 `spin_mode` / `rule_action`。

官方规则动作需要下位机 Control 后续把上位机字段转换为裁判系统 `0x0301` / `0x0120`。

## 1. 场上标准流程

Step 1：建图

Step 2：保存 `map.yaml`

Step 3：回出生点/休息区

Step 4：加载地图启动导航

Step 5：RViz 中 `2D Pose Estimate` 初始化

Step 6：推车或开车到每个战术点，记录 `map` 坐标

Step 7：填写 `sentry_goals.yaml`

Step 8：先用 `mission_runner` 测试固定脚本

Step 9：确认稳定后再启用 `sentry_bt` 全自动决策

## 2. 编译

```bash
cd /home/rm/Desktop/SENTRY_FULL/XMU_RCS_SENTRY
source /opt/ros/humble/setup.bash
rosdep install -r --from-paths src --ignore-src --rosdistro humble -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 3. 建图

建图时建议先不开自瞄，必要时不开串口。

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

确认 `/map` 有数据：

```bash
ros2 topic hz /map
```

保存地图：

```bash
mkdir -p /home/rm/Desktop/SENTRY_FULL/maps
ros2 run nav2_map_server map_saver_cli \
  -f /home/rm/Desktop/SENTRY_FULL/maps/new_map
```

下次导航使用：

```text
map:=/home/rm/Desktop/SENTRY_FULL/maps/new_map.yaml
```

## 4. 点位标定

先加载地图并初始化定位：

```bash
ros2 launch rm_bringup sentry_bringup.launch.py \
  navigation_only:=true \
  debug_no_serial:=true \
  enable_navigation:=true \
  slam:=False \
  map:=/home/rm/Desktop/SENTRY_FULL/maps/new_map.yaml \
  use_rviz:=true
```

在 RViz 里用 `2D Pose Estimate` 给初始位姿。

然后把车推到目标点，读取 `map` 坐标：

```bash
ros2 run tf2_ros tf2_echo map gimbal_yaw_fake
```

记录：

```text
translation.x
translation.y
yaw
```

填写：

```text
src/rm_sentry_decision/config/sentry_goals.yaml
```

示例：

```yaml
BASE_HOME:
  id: 19
  x: 0.15
  y: -0.30
  yaw: 0.0

SUPPLY_LEFT:
  id: 5
  x: 1.80
  y: 1.20
  yaw: 0.0

MID_PRESSURE:
  id: 11
  x: 3.20
  y: 0.40
  yaw: 0.0
```

注意：

1. `sentry_goals.yaml` 里的默认坐标是占位值，不能直接上场用。
2. 所有点都必须在当前 `map` 坐标系下重新量。

## 5. 平时测试：mission_runner 预设脚本模式

适合测试：

1. 去某个点。
2. 到点后停住。
3. 到点后切 Attack / Defense。
4. 到点后进入自瞄火控状态。

配置文件：

```text
src/rm_sentry_decision/config/sentry_mission.yaml
```

示例：

```yaml
mission:
  - name: go_mid_pressure
    goal_id: 11
    tactical_state: 6       # REPOSITION
    posture_intent: 1       # MOVE
    fire_policy: 0          # HOLD_FIRE
    spin_mode: 0
    supercap_mode: 0
    wait_reached: true
    timeout_sec: 20.0

  - name: hold_mid_attack
    goal_id: 11
    tactical_state: 1       # HOLD
    posture_intent: 2       # ATTACK
    fire_policy: 2          # NORMAL
    spin_mode: 1
    supercap_mode: 0
    duration_sec: 999.0
```

启动：

```bash
ros2 launch rm_bringup sentry_bringup.launch.py \
  use_serial:=true \
  serial_device:=/dev/rm_serial \
  baudrate:=460800 \
  enable_navigation:=true \
  enable_vision:=true \
  enable_sentry_command_mux:=true \
  enable_sentry_goal_executor:=true \
  enable_sentry_mission_runner:=true \
  enable_sentry_decision:=false \
  slam:=False \
  map:=/home/rm/Desktop/SENTRY_FULL/maps/new_map.yaml \
  use_rviz:=true
```

逻辑：

```text
mission_runner 发布 /sentry/intent
goal_executor 发 Nav2 goal
goal_executor 判断 tactical reach
/sentry/nav_status.reached=true
mission_runner 进入下一步
command_mux 输出最终 /gimbal_cmd
```

注意：

1. `mission_runner` 是脚本模式，不是真正智能决策。
2. 它适合测试点位、到点、姿态切换。
3. `mission_runner` 和 `sentry_bt` 不能同时作为 `/sentry/intent` 发布者。

## 6. 比赛/全自动：sentry_bt 决策模式

适合完整哨兵逻辑：

1. 低血/低弹 -> 进入补给闭环，按 `SUPPLY_LEFT -> SUPPLY_RIGHT -> BASE_HOME` 候选点寻找 RFID 恢复区。
2. 到达补给候选点后必须用 RFID 确认，`on_supply` 或 `on_base` 成立后才停留等待恢复。
3. 有敌人且状态健康 -> `ENGAGE`，发布内部 `CURRENT_HOLD`（协议 `goal_id=0`）取消导航目标，原地开小陀螺交给自瞄锁敌；血量不健康或当前点位失败则转 `RETREAT`。
4. 热量危险 -> 保守/停火。
5. 巡航/搜索到点 -> dwell 等待一小段时间，再允许切换下一个巡航目标。
6. 死亡且裁判链路新鲜 -> 硬导航到 `BASE_HOME`，停火、关小陀螺，等待 RFID 和回血；`WAIT_REVIVE` 只作为不可移动时的状态语义。

比赛全功能启动命令。队友比赛时优先使用这一条：

```bash
cd /home/rm/Desktop/SENTRY_FULL/XMU_RCS_SENTRY
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch rm_bringup sentry_bringup.launch.py \
  use_serial:=true \
  serial_device:=/dev/rm_serial \
  baudrate:=460800 \
  enable_navigation:=true \
  enable_vision:=true \
  enable_decision:=true \
  enable_sentry_decision:=true \
  enable_sentry_command_mux:=true \
  enable_sentry_goal_executor:=true \
  enable_sentry_mission_runner:=false \
  enable_second_lidar_safety:=true \
  slam:=False \
  initial_pose_yaw:=2.15 \
  map:=/home/rm/Desktop/SENTRY_FULL/maps/new_map.yaml \
  prior_pcd_file:=/home/rm/Desktop/SENTRY_FULL/maps/new_scans.pcd \
  use_rviz:=false
```

如果要做开机自启动 watchdog，直接使用仓库里的 systemd 服务文件：

```bash
sudo install -m 0644 tools/sentry.service /etc/systemd/system/sentry.service
sudo chmod +x tools/start_sentry_watchdog_systemd.sh
sudo systemctl daemon-reload
sudo systemctl enable --now sentry.service
```

停用并清理旧自启：

```bash
sudo systemctl stop sentry.service
sudo systemctl disable sentry.service
sudo rm -f /etc/systemd/system/sentry.service
sudo rm -f /etc/systemd/system/multi-user.target.wants/sentry.service
sudo systemctl daemon-reload
```

注意：

1. `sentry_bt` 和 `mission_runner` 不能同时发布 `/sentry/intent`。
2. 比赛全自动用 `sentry_bt`，因此 `enable_sentry_decision:=true` 且 `enable_sentry_mission_runner:=false`。
3. 平时固定脚本测试点位、姿态、到点流程时才用 `mission_runner`。
4. 这条命令会启动双雷达 safety 链；第二雷达只做速度安全壳，不进入主导航 costmap。
5. 机器人端比赛默认不开 RViz，需要现场看图时临时把 `use_rviz:=true`。
6. `enable_small_gicp` 的 launch 默认值已经是 `false`，比赛命令里不重复写。
7. 目标颜色/敌我识别沿用裁判系统与视觉配置，不在比赛命令里硬编码。
8. 某个普通导航目标 active 超过 `10s` 且未进入战术到点范围时，会标记 stuck/failed，但先把当前位姿等效为临时战术点驻留一轮；驻留期间 `fire_policy=2` 并保持扫描/小陀螺逻辑，驻留结束后再切换备用点。`BASE_HOME` 作为死亡回家硬任务不被 active timeout 切走。

补给/死亡回家相关默认参数在 `sentry_bt.launch.py` 和
`sentry_bt/config/sentry_bt_params.yaml` 中：

```text
rfid.base_bits=[0]
rfid.supply_bits=[19, 20]
rfid.fortress_bits=[17]
rfid.outpost_bits=[18]
rfid.highground_bits=[1, 2, 3, 4, 9, 10, 11, 12]
rfid.base_bits_2=[]
rfid.supply_bits_2=[]
recovery.require_recovery_buff_for_confirm=false
hp_resupply_enter_ratio=0.35
hp_resupply_exit_ratio=0.60
ammo_resupply_enter_count=80
ammo_resupply_exit_count=120
resupply_rfid_confirm_hold_ms=300
resupply_goal_timeout_ms=12000
resupply_wait_recovery_timeout_ms=12000
dead_return_home_enabled=true
dead_return_goal=BASE_HOME
dead_full_hp_exit_ratio=0.98
goal_dwell_search_ms=2500
goal_dwell_hold_ms=3000
spin_hysteresis_enabled=true
spin_on_confirm_ms=0
spin_off_confirm_ms=0
spin_min_on_ms=0
spin_target_hold_ms=5000
spin_under_attack_hold_ms=5000
spin_hp_drop_threshold=0
spin_mux_hold_sec=5.0
```

实车验收重点：

```bash
ros2 topic echo /gimbal_status
ros2 topic echo /sentry_bt/debug
ros2 topic echo /sentry/intent
ros2 topic echo /sentry/nav_status
ros2 topic echo /sentry/nav_goal_debug
```

`/sentry_bt/debug.executor_summary` 会拼出：

```text
need_supply, hp_recovery_active, ammo_recovery_active
on_base, on_supply, on_fortress, on_outpost, on_highground
rfid_status, rfid_status_2, recovery_buff, recovery_confirmed_by
resupply_goal_current, resupply_rfid_confirmed, resupply_waiting_recovery
dead_return_home_active, dead_home_rfid_confirmed, dead_waiting_full_hp
dwell_active, dwell_complete, dwell_remaining_ms
spin_filtered, spin_target_recent, spin_target_last_seen_ms
under_attack, armor_id, hp_deduction_reason
spin_under_attack_recent, spin_under_attack_last_seen_ms
spin_on_since_ms, spin_off_since_ms, spin_last_change_ms
```

现场如果裁判系统 RFID 位变化，复制并修改
`src/sentry_bt/config/sentry_bt_params.yaml`，然后通过 bringup 指向新文件即可：

```yaml
sentry_bt:
  ros__parameters:
    rfid.base_bits: [0]
    rfid.supply_bits: [19, 20]
    rfid.base_bits_2: [1]
    rfid.supply_bits_2: [2, 3]
    recovery.require_recovery_buff_for_confirm: false
```

```bash
ros2 launch rm_bringup sentry_bringup.launch.py \
  ... \
  sentry_bt_params_file:=/home/rm/Desktop/SENTRY_FULL/field_sentry_bt_params.yaml \
  require_recovery_buff_for_confirm:=false
```

`require_recovery_buff_for_confirm:=true` 时，恢复点确认必须同时满足 RFID 和
`recovery_buff > 0`；默认 `false`，避免 RFID 先到、buff 延迟导致赛场上误切点。

`command_mux` 支持 intent-only heartbeat。即使 `/autoaim/gimbal_cmd_raw` 没有发布，
只要 `/sentry/intent` 新鲜，仍会发布安全 `/gimbal_cmd`，让 `goal_id`、复活、
兑血、兑弹、姿态等规则字段下发；此模式下 `fire_control` 强制为 `0`。
调试看：

```bash
ros2 topic echo /sentry/command_mux_debug
```

重点字段：

```text
mode=autoaim_merge / intent_only_heartbeat / idle
has_autoaim_raw, autoaim_fresh, intent_fresh, has_status
final_fire, goal_id, rule_action_type, revive_cmd
remote_hp_req_inc, remote_ammo_req_inc, ammo_exchange_target_total
posture_cmd_referee
```

## 7. 到点判定说明

系统现在不强依赖 Nav2 自己返回 `SUCCEEDED`。

原因：

Nav2 可能已经到目标附近，但仍然 `active`，导致车在目标附近振荡。

解决方式：

`sentry_goal_executor` 自己根据 `map` 坐标判断 tactical reach。默认到目标 `0.45m` 内保持 `0.30s` 后认为战术到点，然后发布 `/sentry/nav_status.reached=true`。必要时 cancel Nav2 goal，让车停止继续追路径。

`sentry_goals.yaml` 支持给单个 goal 配置区域半径和候选点：

```yaml
SUPPLY_LEFT:
  id: 5
  x: 0.0
  y: 0.0
  yaw: 0.0
  radius: 0.8
  candidates:
    - [0.0, 0.0, 0.0]
    - [0.5, 0.0, 0.0]
    - [0.0, 0.5, 0.0]
    - [-0.5, 0.0, 0.0]
    - [0.0, -0.5, 0.0]
```

`candidates` 的每一项是 `[x, y, yaw]`，单位分别是 `m, m, rad`。执行器会从候选点中选一个离当前机器人最近的点发给 Nav2；到达判定使用该 goal 的 `radius`。这可以把“兑换点/巡航点”定义成一个区域，而不是唯一一点。

所以决策层看：

```text
/sentry/nav_status.reached
```

不是看：

```text
/nav_cmd.is_reached
```

`/nav_cmd.is_reached` 仍然保留给下位机或旧链路，但不是决策闭环的主要依据。

## 8. 调试检查

决策和任务：

```bash
ros2 topic echo /sentry/intent
ros2 topic echo /sentry/nav_status
ros2 topic echo /sentry/mission_debug
ros2 topic echo /sentry_bt/debug
ros2 topic echo /sentry/nav_goal_debug
ros2 topic echo /sentry/command_mux_debug
```

自瞄：

```bash
ros2 topic echo /autoaim/target_status
ros2 topic echo /autoaim/debug_fire_gate
ros2 topic echo /gimbal_cmd
```

导航：

```bash
ros2 topic echo /nav_cmd
ros2 topic hz /cmd_vel
ros2 topic hz /terrain_map
ros2 run tf2_ros tf2_echo map gimbal_yaw_fake
```

检查 Nav2 action：

```bash
ros2 action list | grep navigate
```

## 9. 最小验收步骤

A. 点位是否正确

```bash
ros2 run tf2_ros tf2_echo map gimbal_yaw_fake
```

确认当前位置和 `sentry_goals.yaml` 接近。

B. `mission_runner` 是否发 intent

```bash
ros2 topic echo /sentry/intent
```

C. `goal_executor` 是否发目标

```bash
ros2 topic echo /sentry/nav_goal_debug
```

D. 是否战术到点

```bash
ros2 topic echo /sentry/nav_status
```

看到：

```yaml
reached: true
canceled_by_tactical_reach: true
```

说明 tactical reach 生效。

E. 到点后是否切姿态

```bash
ros2 topic echo /gimbal_cmd
```

检查：

```text
state_switch = 2  # Attack
```

或

```text
state_switch = 3  # Defense
```

F. 自瞄敌情是否进入决策

```bash
ros2 topic echo /autoaim/target_status
ros2 topic echo /sentry_bt/debug
```

检查：

```text
has_target
tracking
fire_ready
enemy_in_view
```

## 10. 推荐使用顺序

第一天：

```text
只跑导航 + RViz
建图
标定 sentry_goals.yaml
```

第二步：

```text
开 mission_runner
测：去点、到点、停住、切姿态
```

第三步：

```text
开 vision + command_mux
测：到点后自瞄是否正常
```

第四步：

```text
开 sentry_bt
测：低血/低弹/有敌人时是否切战术态
```

最后：

```text
关闭 RViz
使用 tmux/SSH 低负载运行
```

## 11. 常见问题

Q1：为什么车到点附近还不显示 Nav2 `SUCCEEDED`？

正常。现在主要看 `/sentry/nav_status.reached`。

Q2：为什么 `sentry_bt` 没有自动打敌人？

检查：

```bash
ros2 topic echo /autoaim/target_status
ros2 topic echo /sentry_bt/debug
ros2 topic echo /sentry/command_mux_debug
```

至少需要：

```text
autoaim has_target/tracking/fire_ready 正常
fire_policy 不是 HOLD_FIRE
gimbal_status 中弹量、热量、发射供电正常
```

Q3：为什么低血没有去补给/回家？

检查：

```bash
ros2 topic echo /gimbal_status
ros2 topic echo /sentry_bt/debug
ros2 topic echo /sentry/intent
```

重点看：

```text
current_hp 是否真实
tactical_state 是否变成 RESUPPLY
goal_id 是否变成 SUPPLY_LEFT / SUPPLY_RIGHT / BASE_HOME
rfid_status 是否命中 on_supply/on_base
resupply_waiting_recovery 是否为 true
sentry_goals.yaml 坐标是否正确
```

Q3.1：为什么死亡后不是 WAIT_REVIVE？

这是当前赛场热修的预期行为：死亡且 `match_started=true`、`referee_link_fresh=true` 时，默认认为底盘在冷却后可移动，`/sentry/intent.goal_id` 会强制输出 `BASE_HOME` 对应 id。复活确认命令仍然由 revive 分支并行输出；`WAIT_REVIVE` 不再承担“导航回基地”的语义。

Q4：为什么补血/补弹没有真正生效？

上位机目前只表达规则动作意图。官方远程回血、远程补弹、复活、能量机关需要下位机 Control 把字段转换为裁判系统 `0x0301` / `0x0120`。上位机不能单独保证官方规则动作生效。

Q5：为什么 `mission_runner` 和 `sentry_bt` 不能同时开？

它们都会发布 `/sentry/intent`。同时开会导致两个大脑抢控制。测试脚本用 `mission_runner`，比赛智能用 `sentry_bt`。

---

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
  map:=/home/rm/Desktop/SENTRY_FULL/maps/new_map.yaml \
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
  map:=/home/rm/Desktop/SENTRY_FULL/maps/new_map.yaml \
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
  map:=/home/rm/Desktop/SENTRY_FULL/maps/new_map.yaml \
  enable_small_gicp:=false \
  enable_second_lidar_safety:=true \
  use_rviz:=false
```

### 仅自瞄栈

只启动通信 + 相机 + 识别 + 自瞄，不启动导航。V3 自瞄默认使用
`tracker_backend=csu_tracker`，参数来自
`src/rm_autoaim/config/params.yaml`。该模式默认不开 `command_mux`，
`rm_autoaim` 会直接发布 `/gimbal_cmd`，同时也发布
`/autoaim/gimbal_cmd_raw` 供调试观察。

```bash
ros2 launch rm_bringup sentry_bringup.launch.py \
  vision_only:=true \
  use_serial:=true \
  serial_device:=/dev/rm_serial \
  baudrate:=460800 \
  enable_vision:=true \
  debug_no_serial:=false
```

无串口自瞄调试，适合只接相机/检测链路做 raw PnP 和 tracker 验收：

```bash
ros2 launch rm_bringup sentry_bringup.launch.py \
  vision_only:=true \
  use_serial:=false \
  debug_no_serial:=true \
  enable_vision:=true \
  publish_debug_image:=true
```

启动后确认节点和默认后端：

```bash
ros2 param get /rm_autoaim tracker_backend
ros2 topic hz /detector/armors
ros2 topic echo /autoaim/debug_fire_gate
ros2 topic echo /autoaim/target_status
ros2 topic echo /autoaim/gimbal_cmd_raw
ros2 topic echo /gimbal_cmd
```

V3 几何验收时可以临时切到 raw PnP 后端：

```bash
ros2 param set /rm_autoaim tracker_backend raw_pnp
ros2 topic echo /autoaim/debug_fire_gate
ros2 topic echo /autoaim/target_status
ros2 topic echo /autoaim/gimbal_cmd_raw
```

回到默认 tracker：

```bash
ros2 param set /rm_autoaim tracker_backend csu_tracker
```

### 导航 + 自瞄同时开

完整整车：通信、PB2025 导航、相机、识别、自瞄同时启动。下面这条是
“不启用 command_mux”的直连模式：`rm_autoaim` 直接发布最终
`/gimbal_cmd`。上位机语义保持为：无目标
`mode=0,state_switch=1,fire_control=0`；有目标但不能开火
`mode=1,state_switch=2,fire_control=0`；允许开火才发布
`fire_control=1`。

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
  map:=/home/rm/Desktop/SENTRY_FULL/maps/new_map.yaml \
  enable_small_gicp:=false \
  enable_second_lidar_safety:=false \
  use_rviz:=false
```

需要让决策/command_mux 接管最终 `/gimbal_cmd` 时，打开
`enable_sentry_command_mux`。此时 launch 会把 `rm_autoaim` 的
`/gimbal_cmd` remap 到 `/autoaim/gimbal_cmd_raw`，最终 `/gimbal_cmd`
由 `sentry_command_mux` 发布：

```bash
ros2 launch rm_bringup sentry_bringup.launch.py \
  use_serial:=true \
  serial_device:=/dev/rm_serial \
  baudrate:=460800 \
  enable_navigation:=true \
  enable_vision:=true \
  enable_sentry_command_mux:=true \
  enable_sentry_decision:=true \
  navigation_only:=false \
  vision_only:=false \
  slam:=False \
  map:=/home/rm/Desktop/SENTRY_FULL/maps/new_map.yaml \
  enable_small_gicp:=false \
  enable_second_lidar_safety:=false \
  use_rviz:=false
```

command_mux 模式下重点看：

```bash
ros2 topic echo /autoaim/gimbal_cmd_raw
ros2 topic echo /sentry/intent
ros2 topic echo /sentry/command_mux_debug
ros2 topic echo /gimbal_cmd
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
  map:=/home/rm/Desktop/SENTRY_FULL/maps/new_map.yaml \
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
  map:=/home/rm/Desktop/SENTRY_FULL/maps/new_map.yaml \
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
MAP=/home/rm/Desktop/SENTRY_FULL/maps/new_map.yaml ./tools/start_dual_lidar_full_tmux.sh
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
| `enable_vision` | `true` | 启动相机、识别、自瞄链路 |
| `enable_sentry_command_mux` | `false` | 启动后由 command_mux 发布最终 `/gimbal_cmd`，自瞄输出 remap 到 `/autoaim/gimbal_cmd_raw` |
| `enable_sentry_decision` | `false` | 启动 `sentry_bt` 意图层，通常和 command_mux 一起联调 |
| `sentry_goal_active_timeout_sec` | `10.0` | 单个决策目标 active 超时后标记 stuck/failed，让 BT 切换备用点 |
| `enable_small_gicp` | `false` | 默认关闭 PB small_gicp 重定位 |
| `enable_second_lidar_safety` | `false` | 可选第二雷达速度安全壳，不改变主导航链 |
| `target_color` | `red` | 检测器目标颜色，可启动时覆盖为 `blue` 或 `all` |
| `use_rviz` | `false` | 默认机器人端不开 RViz |
| `slam` | `False` | `False` 使用已有地图，`True` 进入 PB SLAM 模式 |
| `map` | `/home/rm/Desktop/SENTRY_FULL/maps/new_map.yaml` | Nav2 栅格地图 |
| `prior_pcd_file` | `/home/rm/Desktop/SENTRY_FULL/maps/new_scans.pcd` | 可选 PCD 先验 |

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
ros2 param get /rm_autoaim tracker_backend
ros2 topic echo /autoaim/gimbal_cmd_raw
ros2 topic echo /gimbal_cmd
ros2 topic echo /autoaim/debug_fire_gate
ros2 topic echo /autoaim/debug_world_points
```

`/autoaim/debug_fire_gate` 里 V3 重点看：
`backend`、`source`、`raw_pnp_pos`、`tracker_pos`、`selected_pos`、
`manual_yaw_offset`、`manual_pitch_offset`、`tracker_state`、`fire_control`、
`reason`。默认应看到 `backend=csu_tracker`；切 raw PnP 验几何时应看到
`source=raw_pnp`。

完整整车联调时重点看：

```bash
ros2 topic echo /nav_cmd
ros2 topic echo /autoaim/gimbal_cmd_raw
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
