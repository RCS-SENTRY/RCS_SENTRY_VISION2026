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
- PB2025/Nav2 still owns `/cmd_vel`, and the existing PB bridge still converts
  safe velocity output to `/nav_cmd`.

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
ros2 topic echo /sentry_bt/debug
ros2 topic echo /sentry/command_mux_debug
ros2 topic echo /sentry/nav_goal_debug
ros2 topic echo /autoaim/gimbal_cmd_raw
ros2 topic echo /gimbal_cmd
ros2 topic echo /gimbal_status
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

Default startup keeps decision disabled. In that mode, `rm_autoaim` still
publishes `/gimbal_cmd` directly, and the PB2025 navigation chain is unchanged.
