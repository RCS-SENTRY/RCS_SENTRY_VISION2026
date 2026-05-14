#!/usr/bin/env bash
set -euo pipefail

WORKDIR="/home/rm/Desktop/SENTRY_FULL/XMU_RCS_SENTRY"
LOGDIR="$WORKDIR/Log/watchdog"
mkdir -p "$LOGDIR"

TERM_BIN=""
for term in gnome-terminal konsole x-terminal-emulator xterm rxvt; do
  if command -v "$term" >/dev/null 2>&1; then
    TERM_BIN="$(command -v "$term")"
    break
  fi
done

RUN_CMD="while true; do
  cd '$WORKDIR'
  set +u
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  set -u
  ros2 launch rm_bringup sentry_bringup.launch.py \\
    use_serial:=true \\
    serial_device:=/dev/rm_serial \\
    baudrate:=460800 \\
    enable_navigation:=true \\
    enable_vision:=true \\
    enable_decision:=true \\
    enable_sentry_decision:=true \\
    enable_sentry_command_mux:=true \\
    enable_sentry_goal_executor:=true \\
    enable_sentry_mission_runner:=false \\
    sentry_goals_file:='$WORKDIR/src/rm_sentry_decision/config/sentry_goals.yaml' \\
    sentry_bt_params_file:='$WORKDIR/src/sentry_bt/config/sentry_bt_params.yaml' \\
    enable_second_lidar_safety:=true \\
    slam:=False \\
    initial_pose_yaw:=2.15 \\
    map:=/home/rm/Desktop/SENTRY_FULL/maps/new_map.yaml \\
    prior_pcd_file:=/home/rm/Desktop/SENTRY_FULL/maps/new_scans.pcd \\
    use_rviz:=false 2>&1 | tee -a '$LOGDIR/sentry_watchdog.log'
  echo '[watchdog] ros2 launch exited, restarting in 5s...' | tee -a '$LOGDIR/sentry_watchdog.log'
  sleep 5
done"

if [ -z "$TERM_BIN" ]; then
  bash -lc "$RUN_CMD"
  exit 0
fi

case "$(basename "$TERM_BIN")" in
  gnome-terminal)
    exec "$TERM_BIN" -- bash -lc "$RUN_CMD"
    ;;
  konsole)
    exec "$TERM_BIN" --noclose -e bash -lc "$RUN_CMD"
    ;;
  xterm|x-terminal-emulator|rxvt)
    exec "$TERM_BIN" -hold -e bash -lc "$RUN_CMD"
    ;;
  *)
    exec "$TERM_BIN" -e bash -lc "$RUN_CMD"
    ;;
esac
