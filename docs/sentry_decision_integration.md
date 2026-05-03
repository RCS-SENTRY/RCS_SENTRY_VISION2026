# Sentry Decision Integration

This integration keeps the stable PB2025/Nav2 navigation chain and the stable
rm_autoaim chain. The behavior tree is now an intent-layer node only.

## Topic Boundary

- `sentry_bt` publishes `/sentry/intent` and `/sentry_bt/debug`.
- `sentry_bt` does not publish `/gimbal_cmd`.
- `sentry_bt` does not publish `/nav_cmd`.
- `rm_autoaim` still computes yaw, pitch, mode, and raw `fire_control`.
- When command mux is enabled, `rm_autoaim` is remapped from `/gimbal_cmd` to
  `/autoaim/gimbal_cmd_raw`.
- `sentry_command_mux_node` merges `/autoaim/gimbal_cmd_raw`,
  `/sentry/intent`, and `/gimbal_status`, then publishes the final
  `/gimbal_cmd`.
- `sentry_goal_executor_node` maps `SentryIntent.goal_id` to a Nav2
  `NavigateToPose` action goal. It does not publish `/nav_cmd` or `/cmd_vel`.
- `sentry_goal_executor_node` publishes `/sentry/nav_status` for decision-layer
  navigation state. Decision code must not use `/nav_cmd.is_reached` as input.
- `rm_autoaim` publishes `/autoaim/target_status` as structured enemy/aim state.
- PB2025/Nav2 still owns `/cmd_vel`, and the existing PB bridge still converts
  safe velocity output to `/nav_cmd`.

## Intent Sources

There are two supported intent sources:

- `sentry_bt`: strategy mode. It reads referee/status input,
  `/autoaim/target_status`, and `/sentry/nav_status`, then publishes
  `/sentry/intent`.
- `sentry_mission_runner_node`: preset script mode. It reads
  `config/sentry_mission.yaml`, waits on `/sentry/nav_status` when requested,
  and publishes `/sentry/intent`.

Only one of them should publish `/sentry/intent` at a time. The bringup launch
prefers `sentry_bt` when both `enable_sentry_decision` and
`enable_sentry_mission_runner` are set, and disables mission runner with a
warning.

## Tactical Reach

Nav2 may remain active near a target and keep the controller chasing small path
errors. This integration does not modify Nav2 parameters. Instead,
`sentry_goal_executor_node` checks TF from `map` to `gimbal_yaw_fake` and marks
the active goal reached when the robot stays within `tactical_reach_radius`
for `tactical_reach_hold_sec`.

Default values:

- `tactical_reach_radius=0.45`
- `tactical_reach_hold_sec=0.30`
- `cancel_nav2_on_tactical_reach=true`

When tactical reach is satisfied, `/sentry/nav_status.reached=true` is
published and the current Nav2 goal is canceled. This stops PB/Nav2 from
continuing to chase the same nearby target. `/nav_cmd.is_reached` remains a
lower-level signal for Control and is not fed back into the decision layer.

## Goal Table

`src/rm_sentry_decision/config/sentry_goals.yaml` covers every goal name the
BT may output, including `BASE_HOME` and `BASE_HOLD`. The included coordinates
are placeholders. Every `x/y/yaw` must be measured and filled under the actual
competition `map` frame before field use.

## Fire Gating

Final fire is produced only by the mux:

```text
final_fire = autoaim_fire && decision_fire_policy && safety_ok
```

With the default `allow_fire_without_intent=false`, intent timeout forces
`fire_control=0`. The safety gate requires HP, 17 mm ammo allowance, shooter
power output, and heat margin.

## Posture Encoding

There are two posture encodings and they must not be mixed:

- Lower state switch: `Move=1`, `Attack=2`, `Defense=3`
- Referee `posture_cmd_referee`: `Attack=1`, `Defense=2`, `Move=3`

`SentryIntent.posture_intent` uses the lower semantic order:
`KEEP=0`, `MOVE=1`, `ATTACK=2`, `DEFENSE=3`.

## RM2026 Rule Fields

The official sentry autonomous decision command is `cmd_id=0x0301`,
sub-content ID `0x0120`. This upper-computer change only generates clear
semantic fields. Control is still responsible for packing them into the
official bitfield and closing the monotonic counter loop.

The official `0x0120` remote ammo count, remote HP count, and ammo exchange
target are monotonic fields. The mux only passes current-cycle `0/1` pulse
requests for remote ammo, remote HP, revive confirmation, instant revive, and
energy activation. It does not claim those are final official counters.

`claim_periodic_ammo` is not part of official `0x0120`; phase 1 disables it and
forces final `/gimbal_cmd.claim_periodic_ammo=0`.

Until Control is updated, rule action fields are upper-computer semantic
pass-through only. They do not guarantee that the referee system action has
actually taken effect.

## Useful Debug Commands

```bash
ros2 topic echo /sentry/intent
ros2 topic echo /sentry/nav_status
ros2 topic echo /sentry_bt/debug
ros2 topic echo /sentry/command_mux_debug
ros2 topic echo /sentry/nav_goal_debug
ros2 topic echo /sentry/mission_debug
ros2 topic echo /autoaim/target_status
ros2 topic echo /autoaim/gimbal_cmd_raw
ros2 topic echo /gimbal_cmd
ros2 topic echo /gimbal_status
ros2 topic echo /nav_cmd
ros2 action list | grep navigate
```

Bench debug:

```bash
ros2 launch sentry_bt sentry_bt.launch.py mode:=debug
```

Intent-layer robot startup:

```bash
ros2 launch rm_bringup sentry_bringup.launch.py \
  enable_decision:=true \
  enable_sentry_decision:=true \
  enable_sentry_command_mux:=true \
  enable_sentry_goal_executor:=true
```

Manual goal test:

```bash
ros2 topic pub /sentry/intent rm_interfaces/msg/SentryIntent "{
  protocol_version: 1,
  tactical_state: 6,
  goal_id: 11,
  posture_intent: 1,
  fire_policy: 0,
  spin_mode: 0,
  supercap_mode: 0,
  rule_action_type: 0,
  reason: 'manual test: go MID_PRESSURE'
}" --once
```

Mission runner test:

```bash
ros2 launch rm_bringup sentry_bringup.launch.py \
  enable_navigation:=true \
  enable_vision:=true \
  enable_sentry_command_mux:=true \
  enable_sentry_goal_executor:=true \
  enable_sentry_mission_runner:=true \
  use_rviz:=true
```

Default startup keeps decision disabled. In that mode, `rm_autoaim` still
publishes `/gimbal_cmd` directly, and the PB2025 navigation chain is unchanged.

This round intentionally does not modify:

- `src/rm_bringup/config/pb2025_xmu_nav_params.yaml`
- Nav2 `controller_server`
- Nav2 `goal_checker`
- `FollowPath`
- `velocity_smoother`
- `pb_cmd_vel_to_nav_cmd.py`
