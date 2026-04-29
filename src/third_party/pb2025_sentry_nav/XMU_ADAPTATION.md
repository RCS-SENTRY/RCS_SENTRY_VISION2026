# XMU Adaptation Notes

This vendored PB2025 navigation stack is used as the only navigation path in
`XMU_RCS_SENTRY`.

XMU-specific integration is kept outside the upstream PB2025 packages:

- `rm_bringup/launch/sentry_pb2025_takeover.launch.py` starts the PB2025 chain.
- `rm_bringup/config/pb2025_xmu_nav_params.yaml` provides the MID-360 network,
  frame, Nav2, terrain, and chassis overrides.
- `rm_bringup/config/pb2025_xmu_frames.yaml` documents the runtime TF tree.
- `rm_bringup/config/pb2025_xmu_vehicle.yaml` records the XMU sentry dimensions.
- `rm_bringup/scripts/pb_cmd_vel_to_nav_cmd.py` bridges PB `Twist` output to the
  existing XMU `rm_interfaces/msg/NavCmd` serial interface.

The XMU real MID-360 mounting transform is:

- `lidar_x = 0.0`
- `lidar_y = 0.2`
- `lidar_z = 0.35`
- `lidar_roll = 0.0`
- `lidar_pitch = 0.3115`
- `lidar_yaw = 1.5708`

The XMU chassis dimensions are:

- `length = 0.50 m`
- `width = 0.50 m`
- `height = 0.55 m`

The upstream PB2025 `LICENSE` and `README.md` are preserved in this directory.
