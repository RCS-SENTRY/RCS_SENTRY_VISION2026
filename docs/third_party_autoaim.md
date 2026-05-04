# Third-party Autoaim Notes

`rm_autoaim` V3 ports and adapts the autoaim core ideas from
`CSU-FYT-Vision/FYT2024_vision` while keeping the RCS ROS shell, topics,
messages, calibration parameters, bringup integration, command mux, decision
nodes, and hardware bridge.

Ported/adapted modules:

- `armor_detector/armor_pose_estimator`: IPPE/PNP multi-solution handling,
  reprojection-error sanity checks, image-tilt based solution disambiguation,
  and small/large armor object models. RCS intrinsics/extrinsics are used.
- `armor_solver/armor_tracker`: FYT-style target state
  `[xc, vxc, yc, vyc, zc, vzc, yaw, v_yaw, r, d_zc]`, armor measurement model,
  armor jump handling, target persistence, and
  `LOST / DETECTING / TRACKING / TEMP_LOST` state transitions. The RCS adapter
  uses a local Eigen EKF with numeric Jacobians instead of importing FYT
  `rm_utils`/Ceres headers.
- `rm_utils/math/manual_compensator`: distance-based yaw/pitch manual
  compensation. RCS uses a linear-interpolated, clamped LUT parameter format.
- `armor_solver` aiming/fire-window ideas: final command generation still
  writes RCS `rm_interfaces/msg/GimbalCmd`.

Not imported:

- FYT bringup, launch, camera driver, serial driver, navigation, decision,
  messages, or calibration files.
- FYT hard-coded camera intrinsics/extrinsics.
- FYT Ceres/rm_utils build chain; the tracker EKF math is adapted locally to
  keep `rm_autoaim` dependency-light.

License:

- FYT source files used as references carry Apache-2.0 headers, with tracker
  lineage noting MIT-origin code plus Apache-2.0 modifications. The RCS V3
  adapter files retain these attribution headers where FYT logic was adapted.
