# RoboMaster 哨兵 BT 热修计划

范围：`RCS-SENTRY/RCS_SENTRY_VISION2026` 比赛前热修。保持下位机协议和 ROS msg 字段不变，不做大重构。

## 目标

- 保留裁判系统姿态收益：`ATTACK=1`、`DEFENSE=2`、`MOVE=3`。
- 防止姿态位、`spin_mode`、`fire_policy` 快速跳变。
- 速度完全交给导航栈、速度控制层和双雷达避障链。
- 自瞄扫描、导航巡航、小陀螺并行运行。
- 不改下位机协议，不新增 ROS msg 字段。

## 必须保持的语义

- `fire_policy=2`：允许自瞄扫描和 autoaim raw cmd 通路。
- `fire_policy=1`：关闭自瞄扫描和 autoaim raw cmd 通路。
- `fire_policy` 不表示“已经发现目标”。
- 最终发弹仍由 autoaim raw fire control、数据新鲜度、枪管热量、弹量、裁判发弹许可共同决定。
- 枪管热量只限制最终发弹，不能改变战术态、目标点、自瞄扫描许可或小陀螺。
- 导航失败、到点超时、避障、重规划不等于裁判系统异常。
- 姿态不得反向控制导航、开火策略或小陀螺。
- 小陀螺不得依赖姿态或 `internal_motion_latched`。
- 出现目标不得导致 goal 切到 `CURRENT_HOLD`。
- LLM advice 不得影响 tactical state、goal、posture、fire、spin、supercap。

## 工作项

### 1. 基线检查

- [x] 检查 `src/sentry_bt/src/bt_nodes_executor.cpp`。
- [x] 检查 `src/sentry_bt/src/bt_nodes_tactical.cpp`。
- [x] 检查 `src/sentry_bt/include/robot_context.hpp`。
- [x] 检查 `src/sentry_bt/config/sentry_bt_params.yaml`。
- [x] 检查 `src/sentry_bt/src/nav_interface.cpp`。
- [x] 检查 `src/sentry_bt/include/nav_interface.hpp`。
- [x] 检查 `src/sentry_bt/main.cpp`。
- [x] 检查 `src/sentry_bt/launch/*.launch.py`。
- [x] 检查 `src/rm_sentry_decision/src/sentry_command_mux_node.cpp`。
- [x] 检查 `src/rm_sentry_decision/config/sentry_goals.yaml`。
- [x] 记录现有硬编码 `vx/vy/wz`、`CURRENT_HOLD`、`heat_ratio -> RETREAT`、LLM advice、`internal_motion_latched` 决策链路。

基线记录：

- `main.cpp` 当前已是 intent-only，tick 后写入 `last_nav_command="disabled..."`，不主动发布 `/nav_cmd`。
- `nav_interface.cpp` 仍存在 `GoalVelocity()` 和 `/decision/nav_cmd` publisher，可被其他入口误用，需改成 fake/debug only 且默认不发布。
- `bt_nodes_executor.cpp` 中 `DefaultGoalForState(ENGAGE)` 返回 `CURRENT_HOLD`，`ApplyPostureDecisionNode` 中 ENGAGE 直接 ATTACK，且 fire/spin/supercap 依赖 `internal_motion_latched`。
- `bt_nodes_tactical.cpp` 中 `EvaluateTacticalStateNode` 仍会因 `heat_ratio > 0.90` RETREAT、因 enemy confidence 进入 ENGAGE/REPOSITION、因 LLM advice 改 tactical state。
- `EvaluateEngageGoalsNode` 明确发布 `CURRENT_HOLD`。
- `SetCombatPosturePreferenceNode`、`SetCombatSupercapPreferenceNode` 仍读取 `llm_advice`，`AdviceBonus()` 仍给 goal 加分。
- mux 当前 raw 通路由 `fire_policy >= 2` 放行，`final_fire` 由 autoaim fresh/raw fire/safety 控制；但 spin 仍有 lidar/safety 参数残留，需要确认不因普通避障关 spin。

### 2. 禁用 BT 正式速度输出

- [x] 确保正式 `sentry_bt` 链路只发布 goal 或 intent，不发布硬编码速度。
- [x] 如有必要，保留 `GoalVelocity()` 作为 fake/debug 工具。
- [x] 添加明确注释：比赛正式链路禁止使用 `GoalVelocity()`。
- [x] 禁用 BT 默认正式发布 `/decision/nav_cmd` 速度的路径。
- [x] 确认 launch 默认不启用 fake/debug 速度输出。

### 3. 简化战术态

- [x] 修改 `EvaluateTacticalStateNode`，优先级为：
  - `!match_started`：SEARCH 或安全待机行为，但不允许发弹。
  - 裁判系统不可用或异常：RETREAT 或安全态。
  - 死亡：RETREAT，保留 revive 分支。
  - 真正紧急急停：RETREAT。
  - 低血或需要补给：RESUPPLY。
  - 其他：SEARCH。
- [x] 移除由目标置信度或目标可见性触发的 ENGAGE 抢占。
- [x] 移除 `heat_ratio` 触发 RETREAT。
- [x] 导航异常不得被归类为裁判或发弹安全异常。
- [x] 热修期间不主动进入 REPOSITION。

### 4. 修正 Fire Policy 语义

- [x] 修改 `ApplyFireDecisionNode` 和/或 `UpdateCombatFirePolicyNode`，使 SEARCH、dwell、保留的 ENGAGE、RETREAT 默认输出 `fire_policy=2`。
- [x] RESUPPLY、DEAD、比赛未开始、裁判不可用或异常、发弹合法性不确定时输出 `fire_policy=1`。
- [x] 不使用 `enemy_confidence`、`autoaim_has_target`、`autoaim_tracking`、`autoaim_fire_ready` 关闭自瞄扫描。
- [x] 不使用 `internal_motion_latched` 决定 `fire_policy`。
- [x] 导航失败、`nav_goal_failed`、避障重规划不得关闭自瞄扫描。

### 5. 稳定 Spin Mode

- [x] 修改 `ApplySpinDecisionNode`，忽略姿态和 `internal_motion_latched`。
- [x] SEARCH、dwell、保留的 ENGAGE、RETREAT、避障、goal failed、重规划时 spin ON。
- [x] RESUPPLY、DEAD、比赛未开始、裁判不可用或异常、真正紧急急停时 spin OFF。
- [x] 如已有 hysteresis 参数则保留，但主逻辑必须按稳定状态策略输出。
- [x] 确保 supercap guard 不反向造成 spin 快速跳变。

### 6. 姿态候选与锁存

- [x] 新增或复用上下文字段：
  - `posture_candidate`
  - `posture_candidate_since_ms`
  - `posture_candidate_confirm_ms`
  - `posture_min_hold_ms`
  - `attack_enter_confirm_ms`
  - `defense_enter_confirm_ms`
  - `move_enter_confirm_ms`
  - `nav_goal_timeout_as_temp_defense_ms`
- [x] 保留现有 Posture enum 和姿态累计/debuff 字段。
- [x] 在 `ApplyPostureDecisionNode` 中实现候选评分/选择。
- [x] 硬安全候选：死亡、比赛未开始、裁判不可用或异常、RESUPPLY、need supply、RETREAT 时为 MOVE。
- [x] 移动候选：nav goal active 且未到点时倾向 MOVE，但不绕过保持时间/冷却强切。
- [x] 到点驻留候选：dwell 或 reached 稳定确认后才允许 DEFENSE。
- [x] 临时防守候选：goal 12s 未到点时可考虑 DEFENSE，同时保持 nav active、自瞄扫描 ON、spin ON。
- [x] 接敌候选：`autoaim_tracking` 或 `autoaim_fire_ready` 稳定 100-200ms 后才允许 ATTACK。
- [x] 只有满足 candidate 不同于输出、冷却允许、无 pending、候选确认时间已到、当前姿态最小保持时间已到，才切换输出。
- [x] 保留裁判系统 5s 姿态冷却。
- [x] debuff 只影响评分，不允许直接瞬时切姿态。

### 7. 巡航与 Dwell

- [x] 确认 `patrol_goals: ["SEARCH_AREA_B", "MID_CROSS", "SEARCH_AREA_A"]`。
- [x] 确认 id 解析为 `15 -> 18 -> 14`。
- [x] 设置或确认 `goal_dwell_search_ms: 12000`。
- [x] 设置或确认 `goal_dwell_hold_ms: 12000`。
- [x] 目标出现时不改变当前 goal。
- [x] 确保不存在 ENGAGE 发布 `CURRENT_HOLD` 的路径。
- [x] 12s 未到点时保持 nav active 或触发重规划，但不关闭自瞄扫描/小陀螺。

### 8. 补给逻辑

- [x] 确认 goals 配置中 `SUPPLY_LEFT id=5`、`SUPPLY_RIGHT id=6`。
- [x] 低血或需要补给时进入 RESUPPLY。
- [x] RESUPPLY 使用 MOVE 姿态候选/输出，`fire_policy=1`，`spin_mode=0`。
- [x] 超时或失败 10s 后在补给候选点之间切换。
- [x] 到点但未回血且 RFID 未确认时切换候选点。
- [x] HP 恢复到 `hp_resupply_exit_ratio` 后退出 RESUPPLY。
- [x] 本次热修不增加“RFID 确认但等待回血超时”的逻辑。

### 9. 简化 Supercap

- [x] SEARCH、RETREAT、保留的 ENGAGE 使用稳定 KEEP 或 BURST，优先 KEEP。
- [x] RESUPPLY、DEAD 使用 OFF。
- [x] 防止 supercap 策略反向影响 spin/nav/fire。
- [x] `SetCombatSupercapPreferenceNode` 旁路 LLM advice。

### 10. Command Mux 检查

- [x] 保留 `decision_enables_autoaim = fire_policy >= 2`。
- [x] 确保 `fire_policy=2` 时，即使当前无目标，也放行 autoaim raw cmd 通路。
- [x] 确保 `fire_policy=1` 时关闭扫描/raw 通路。
- [x] 确保 `final_fire` 需要 autoaim raw fire control、autoaim fresh、heat margin、弹量、裁判发弹许可。
- [x] RESUPPLY/DEAD 强制 `final_fire=false`。
- [x] raw 通路使能不得绑定目标是否存在。

### 11. LLM 旁路

- [x] 让 `AdviceBonus` 返回 `0`。
- [x] 确保 `EvaluateTacticalStateNode` 不读取 `llm_advice`。
- [x] 确保姿态偏好不读取 `llm_advice`。
- [x] 确保超电偏好不读取 `llm_advice`。
- [x] 如果删除文件会增加编译风险，则保留文件/结构体但不使用。

### 12. 参数

- [x] `hp_resupply_enter_ratio: 0.65`
- [x] `hp_resupply_exit_ratio: 0.98`
- [x] `resupply_goal_timeout_ms: 10000`
- [x] `resupply_candidate_switch_cooldown_ms: 1000`
- [x] `posture_switch_cooldown_ms: 5000`
- [x] `posture_feedback_stable_ms: 200`
- [x] `posture_candidate_confirm_ms: 1000`
- [x] `posture_min_hold_ms: 5000`
- [x] `attack_enter_confirm_ms: 150`
- [x] `defense_enter_confirm_ms: 1000`
- [x] `move_enter_confirm_ms: 300`
- [x] `nav_goal_timeout_as_temp_defense_ms: 12000`

## 验证清单

- [x] 按工作空间约定选择编译命令。
- [x] 编译成功。
- [x] 正式链路没有发布硬编码 `vx/vy/wz`。
- [x] 没有逻辑因为 `enemy_confidence`、`autoaim_has_target` 或跟踪丢失关闭 `fire_policy=2`。
- [x] 没有逻辑因为 posture 或 `internal_motion_latched` 改变 `spin_mode`。
- [x] 没有逻辑因为 posture 或 `internal_motion_latched` 改变 `fire_policy`。
- [x] 没有 ENGAGE 发布 `CURRENT_HOLD`。
- [x] 没有 `heat_ratio -> RETREAT`。
- [x] 没有导航失败或重规划路径关闭自瞄扫描或小陀螺。
- [x] 没有 LLM advice 影响主决策输出。

## Topic 验证记录

仿真或实车观察：

- `/sentry/intent`
- `/gimbal_cmd`
- `/sentry_bt/debug`
- `/sentry/command_mux_debug`
- `/sentry/nav_status`
- 本仓库使用的 autoaim target status topic

期望行为：

- 巡航 goal id 按 `15 -> 18 -> 14` 循环。
- SEARCH/dwell/接敌保持 `fire_policy=2`。
- SEARCH/dwell/接敌/避障保持 `spin_mode=1`。
- 目标出现不改变 `goal_id`，不发布 `CURRENT_HOLD`。
- 枪管热量高只禁止 `final_fire`。
- 导航失败或重规划时仍保持自瞄扫描和小陀螺。
- 低血进入 `SUPPLY_LEFT/RIGHT`，超时或失败 10s 后切候选。
- 到达补给点但未回血且 RFID 未确认时切候选。
- 死亡状态下，满足复活条件时仍正常输出 revive command。
