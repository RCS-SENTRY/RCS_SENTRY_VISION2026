#!/usr/bin/env bash
set -euo pipefail

WORKDIR="/home/rm/Desktop/SENTRY_FULL/XMU_RCS_SENTRY"
LOGDIR="${XDG_STATE_HOME:-/home/rm/.local/state}/xmu_sentry_desktop_watchdog"
LOCKFILE="/tmp/xmu_sentry_desktop_watchdog.lock"
mkdir -p "$LOGDIR"

open_terminal() {
  local self="$1"
  for term in gnome-terminal konsole x-terminal-emulator xterm rxvt; do
    if command -v "$term" >/dev/null 2>&1; then
      case "$term" in
        gnome-terminal)
          exec "$term" -- bash -lc "'$self' --run; exec bash"
          ;;
        konsole)
          exec "$term" --noclose -e bash -lc "'$self' --run"
          ;;
        xterm|x-terminal-emulator|rxvt)
          exec "$term" -hold -e bash -lc "'$self' --run"
          ;;
        *)
          exec "$term" -e bash -lc "'$self' --run"
          ;;
      esac
    fi
  done
}

if [[ "${1:-}" != "--run" ]]; then
  open_terminal "$0"
  exec "$0" --run
fi

exec 9>"$LOCKFILE"
if ! flock -n 9; then
  echo "[desktop-watchdog] another watchdog is already running; exit."
  exit 0
fi

cd "$WORKDIR"

set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

while true; do
  echo "[desktop-watchdog] $(date '+%F %T') starting full sentry chain..." \
    | tee -a "$LOGDIR/sentry_desktop_watchdog.log"

  ros2 launch rm_bringup sentry_bringup.launch.py \
    use_serial:=true \
    serial_device:=/dev/rm_serial \
    baudrate:=460800 \
    navigation_only:=false \
    vision_only:=false \
    debug_no_serial:=false \
    enable_navigation:=true \
    enable_vision:=true \
    enable_decision:=true \
    enable_sentry_decision:=true \
    enable_sentry_command_mux:=true \
    enable_sentry_goal_executor:=true \
    enable_sentry_mission_runner:=false \
    enable_autoaim_target_status:=true \
    sentry_intent_topic:=/sentry/intent \
    sentry_nav_status_topic:=/sentry/nav_status \
    autoaim_target_status_topic:=/autoaim/target_status \
    autoaim_raw_cmd_topic:=/autoaim/gimbal_cmd_raw \
    final_gimbal_cmd_topic:=/gimbal_cmd \
    sentry_goals_file:=/home/rm/Desktop/SENTRY_FULL/XMU_RCS_SENTRY/src/rm_sentry_decision/config/sentry_goals.yaml \
    sentry_bt_params_file:=/home/rm/Desktop/SENTRY_FULL/XMU_RCS_SENTRY/src/sentry_bt/config/sentry_bt_params.yaml \
    sentry_goal_active_timeout_sec:=10.0 \
    target_color:=red \
    color_ignore:=1 \
    publish_debug_image:=false \
    slam:=False \
    map:=/home/rm/Desktop/SENTRY_FULL/maps/new_map.yaml \
    prior_pcd_file:=/home/rm/Desktop/SENTRY_FULL/maps/new_scans.pcd \
    use_sim_time:=false \
    autostart:=true \
    use_respawn:=False \
    log_level:=info \
    use_rviz:=false \
    enable_small_gicp:=false \
    enable_prior_pcd:=false \
    initial_pose_yaw:=2.15 \
    cmd_vel_input_topic:=/cmd_vel \
    cmd_vel_safe_topic:=/cmd_vel_safe \
    nav_cmd_output_topic:=/nav_cmd \
    publish_rate_hz:=20.0 \
    cmd_vel_timeout_sec:=0.25 \
    goal_reached_latch_sec:=1.0 \
    force_zero_angular_z:=true \
    invert_linear_y:=false \
    enable_second_lidar_safety:=true \
    second_lidar_obstacle_timeout_sec:=0.50 \
    second_lidar_emergency_distance:=0.55 \
    second_lidar_slow_distance:=0.85 \
    second_lidar_caution_distance:=1.25 \
    second_lidar_caution_speed_scale:=0.55 \
    second_lidar_slow_speed_scale:=0.25 \
    second_lidar_stop_on_any_emergency:=false \
    second_lidar_global_scale_on_any_obstacle:=false \
    second_lidar_pass_through_when_no_cloud:=false \
    2>&1 | tee -a "$LOGDIR/sentry_desktop_watchdog.log"

  echo "[desktop-watchdog] $(date '+%F %T') launch exited; restart in 5s." \
    | tee -a "$LOGDIR/sentry_desktop_watchdog.log"
  sleep 5
done
