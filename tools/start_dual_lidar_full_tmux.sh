#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/rm/Desktop/SENTRY_FULL/XMU_RCS_SENTRY}"
SESSION="${SESSION:-sentry_dual}"
MAP="${MAP:-/home/rm/Desktop/SENTRY_FULL/maps/self_filtered_map.yaml}"
SERIAL_DEVICE="${SERIAL_DEVICE:-/dev/rm_serial}"
BAUDRATE="${BAUDRATE:-460800}"
USE_RVIZ="${USE_RVIZ:-false}"
USE_SERIAL="${USE_SERIAL:-true}"
ENABLE_SMALL_GICP="${ENABLE_SMALL_GICP:-false}"
BRINGUP_ARGS="${BRINGUP_ARGS:-}"

if ! command -v tmux >/dev/null 2>&1; then
  echo "tmux is not installed. Install it first: sudo apt install tmux" >&2
  exit 1
fi

if [[ ! -d "$WORKSPACE" ]]; then
  echo "Workspace not found: $WORKSPACE" >&2
  exit 1
fi

if [[ ! -f "$MAP" ]]; then
  echo "Map file not found: $MAP" >&2
  exit 1
fi

if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "tmux session '$SESSION' already exists."
  echo "Attach with: tmux attach -t $SESSION"
  echo "Stop it with: tmux kill-session -t $SESSION"
  exit 1
fi

BASE_SETUP="cd '$WORKSPACE' && source /opt/ros/humble/setup.bash && source install/setup.bash"

tmux new-session -d -s "$SESSION" -n full_bringup
tmux send-keys -t "$SESSION:full_bringup" \
  "$BASE_SETUP && ros2 launch rm_bringup sentry_bringup.launch.py use_serial:=$USE_SERIAL serial_device:=$SERIAL_DEVICE baudrate:=$BAUDRATE enable_navigation:=true navigation_only:=false vision_only:=false slam:=False map:=$MAP enable_small_gicp:=$ENABLE_SMALL_GICP enable_second_lidar_safety:=true use_rviz:=$USE_RVIZ $BRINGUP_ARGS" C-m

tmux new-window -t "$SESSION" -n monitor
tmux send-keys -t "$SESSION:monitor" \
  "$BASE_SETUP && clear && echo 'Dual lidar full system monitor' && echo && echo 'This uses one Livox driver in multi-topic CustomMsg mode.' && echo && echo 'Useful checks:' && echo '  ros2 topic hz /livox/lidar_192_168_1_173' && echo '  ros2 topic hz /livox/lidar_192_168_1_166' && echo '  ros2 topic hz /second_livox/lidar' && echo '  ros2 topic hz /second_lidar_obstacle_cloud' && echo '  ros2 topic hz /cmd_vel_safe' && echo '  ros2 topic hz /nav_cmd' && echo '  ros2 topic echo /second_lidar_safety_debug --once' && echo && bash" C-m

tmux select-window -t "$SESSION:full_bringup"

cat <<EOF
Started tmux session: $SESSION

Windows:
  1 full_bringup  - full sentry bringup with one Livox driver and enable_second_lidar_safety:=true
  2 monitor       - shell with common check commands

Attach:
  tmux attach -t $SESSION

Stop all:
  tmux kill-session -t $SESSION

Common overrides:
  MAP=/path/to/map.yaml USE_RVIZ=true $0
  SERIAL_DEVICE=/dev/ttyUSB1 $0
  BRINGUP_ARGS='use_rviz:=true log_level:=debug' $0
EOF
