# RMUC2026 哨兵决策规则约束审计

本文件记录当前上位机决策层与 RM2026 哨兵相关规则/通信约束的静态审计结果。它用于合并前检查，不代表官方规则动作已经由下位机 Control 完成闭环。

## 结论

当前决策层已经满足大部分上位机语义约束：

- `sentry_bt` 只发布 `/sentry/intent` 和 `/sentry_bt/debug`，不直接发布 `/gimbal_cmd` 或 `/nav_cmd`。
- `command_mux` 将自瞄 raw cmd 与决策 intent 合成最终 `/gimbal_cmd`。
- `goal_executor` 只向 Nav2 发送 NavigateToPose goal，不直接发布速度。
- `claim_periodic_ammo` 最终输出被上位机阶段置 0。
- 远程补弹/远程回血在上位机侧只输出 0/1 脉冲，最终官方 0x0120 单调计数仍应由 Control 闭环。
- 姿态编码在 `command_mux` 内区分了下位机 `state_switch` 与官方 `posture_cmd_referee`。

但仍存在以下规则风险，合并到比赛主线前应修复。

## P0：堡垒不应作为普通允许发弹量兑换点

### 当前风险

`src/sentry_bt/src/bt_nodes_rule.cpp` 中，普通补弹点判断包含：

```cpp
ctx.on_supply || ctx.on_base || ctx.on_outpost || ctx.on_fortress
```

风险位置：

- `EvaluateRuleAction()` 的 `on_projectile_exchange_point`
- `ExchangeAmmoAtPointNode::tick()` 的合法点位判断

RM2026 语义中，普通允许发弹量兑换点应按补给区/基地增益点/前哨站增益点处理。堡垒对应的是堡垒增益/储备允许发弹量，不应被当成普通 `EXCHANGE_AMMO_AT_POINT` 触发点。

### 建议修复

把普通补弹点收窄为：

```cpp
const bool on_projectile_exchange_point =
    ctx.on_supply || ctx.on_base || ctx.on_outpost;
```

并同步修改 `ExchangeAmmoAtPointNode::tick()`：

```cpp
if (!(ctx.on_supply || ctx.on_base || ctx.on_outpost) || ctx.gold < 10)
{
    return BT::NodeStatus::FAILURE;
}
```

若以后需要堡垒储备允许发弹量，应新增独立规则动作或状态字段，不要复用普通兑换动作。

## P1：行为树 XML 仍保留 `CLAIM_PERIODIC_AMMO` 分支

### 当前状态

当前上位机主循环会在发布 intent 前强制：

```cpp
ctx.claim_periodic_ammo = 0;
```

`command_mux` 也会强制：

```cpp
out.claim_periodic_ammo = 0;
```

所以该字段不会真正下发。

### 风险

`src/sentry_bt/tree/sentry_main.template.xml` 的 `RuleActionSubtree` 仍包含：

```xml
case_5="CLAIM_PERIODIC_AMMO"
<ClaimPeriodicAmmo/>
```

这会增加 debug 误导：队员可能以为该动作是官方 0x0120 的一部分。

### 建议修复

第一阶段建议从 XML 中移除 `CLAIM_PERIODIC_AMMO` case，或保留节点但不接入主树，并在文档中标为本地扩展/禁用。

## P1：规则动作尚未完成官方 0x0301 / 0x0120 闭环

上位机可以输出：

- `request_remote_ammo_once`
- `request_remote_hp_once`
- `request_revive_confirm`
- `request_instant_revive`
- `request_activate_energy`
- `posture_cmd_referee`

但官方 `0x0301` / 子内容 ID `0x0120` 的 bitfield 发送、远程补弹/远程回血单调计数、兑换发弹量累计值单调不减，仍需要 Control 下位机实现并回传状态。当前上位机不能单独保证官方规则动作真实生效。

## P2：仿真逻辑也应跟随规则收窄堡垒普通兑换

`src/sentry_bt/src/sentry_bt_sim_main.cpp` 的 `ApplyAmmoTarget()` 同样允许 `sim_input_.on_fortress` 触发普通兑换。建议与生产逻辑保持一致，避免仿真给出错误正例。

## 建议回归检查

新增或手工执行以下检查：

```bash
grep -R "on_supply || ctx.on_base || ctx.on_outpost || ctx.on_fortress" -n src/sentry_bt src/rm_sentry_decision
grep -R "CLAIM_PERIODIC_AMMO" -n src/sentry_bt/tree
```

期望：

- 普通 `EXCHANGE_AMMO_AT_POINT` 不再包含 `on_fortress`。
- 第一阶段主树不再直接接入 `CLAIM_PERIODIC_AMMO`。

## 合并建议

本审计 PR 建议后续补一个代码修复 PR，内容包括：

1. 从普通发弹量兑换点判断中移除 `on_fortress`。
2. 从 `RuleActionSubtree` 中移除 `CLAIM_PERIODIC_AMMO` 分支。
3. 同步修正 `sentry_bt_sim_main.cpp` 的普通兑换仿真。
4. 更新 README 中“堡垒储备发弹量不等同普通兑换点”的说明。
