# Autoaim V3 Test Flow

## raw_pnp geometry check

```bash
ros2 param set /rm_autoaim tracker_backend raw_pnp
ros2 topic echo /autoaim/debug_fire_gate
ros2 topic echo /autoaim/target_status
ros2 topic echo /autoaim/gimbal_cmd_raw
```

Expected:

- Static 2 m armor gives stable yaw/pitch.
- `debug_fire_gate` shows `source=raw_pnp`.
- No `NaN` in position, angle, or distance fields.

## csu_tracker default chain

```bash
ros2 param set /rm_autoaim tracker_backend csu_tracker
ros2 topic echo /autoaim/debug_fire_gate
ros2 topic echo /autoaim/target_status
```

Expected:

- Tracker reaches `TRACKING` after the configured detect count.
- `TEMP_LOST` recovers quickly when detections return.
- `selected_pos` stays on the shootable armor point.
- `fire_control` opens only when aligned, within distance, and not using unsafe fallback.

## pitch LUT tuning

Watch these fields in `/autoaim/debug_fire_gate`:

- `distance`
- `manual_pitch_offset`
- `pitch_cmd`
- `pitch_err_deg`

Adjust `manual_compensator.pitch_lut` in `src/rm_autoaim/config/params.yaml` as flat `[distance, offset, ...]` pairs. ROS 2 Humble does not reliably accept nested numeric arrays in parameter files.

