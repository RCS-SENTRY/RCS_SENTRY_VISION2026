#!/usr/bin/env bash
set -euo pipefail

WORKDIR="/home/rm/Desktop/SENTRY_FULL/XMU_RCS_SENTRY"
LOGDIR="$WORKDIR/Log/systemd"
mkdir -p "$LOGDIR"
cd "$WORKDIR"

set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

exec ros2 launch rm_bringup sentry_bringup.launch.py \
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
  use_rviz:=false \
  >>"$LOGDIR/sentry_match.log" 2>&1
