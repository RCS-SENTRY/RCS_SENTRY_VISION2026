#!/bin/bash
# =============================================================================
# tune_ukf.sh — IMM-UKF 一键调参脚本 (闭环版)
# =============================================================================
# 需要 4 个终端:
#   终端1: ./tools/tune_ukf.sh play        # 播放 bag (仅检测器数据)
#   终端2: ./tools/tune_ukf.sh fake         # 闭环假云台 (提供 IMU + 跟踪 GimbalStatus)
#   终端3: ./tools/tune_ukf.sh run          # 启动 autoaim
#   终端4: ./tools/tune_ukf.sh plot         # PlotJuggler 可视化
#
# 改参数后:
#   终端3: Ctrl+C → ./tools/tune_ukf.sh rebuild
# =============================================================================

WORKSPACE="/home/rm/Desktop/SENTRY_FULL/XMU_RCS_SENTRY"
BAG_PATH="/home/rm/bag2/autoaim_test_20260406_114723"
PARAMS_FILE="$WORKSPACE/rm_autoaim/config/params.yaml"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"

case "$1" in

  # ===========================================================================
  # 循环播放 bag (仅检测器数据, IMU 和 Status 由 fake_sensor 提供)
  # ===========================================================================
  play)
    echo -e "${GREEN}=== 循环播放 Rosbag (仅 /detector/armors) ===${NC}"
    echo -e "${CYAN}IMU + GimbalStatus 由 fake_sensor 闭环提供${NC}"
    echo -e "${CYAN}按 Ctrl+C 停止${NC}"
    echo ""
    ros2 bag play "$BAG_PATH" \
      --topics /detector/armors \
      --clock \
      --loop \
      -r 1.0
    ;;

  # ===========================================================================
  # 循环播放 bag (全部话题, 兼容旧方式)
  # ===========================================================================
  play-full)
    echo -e "${GREEN}=== 循环播放 Rosbag (全部话题) ===${NC}"
    ros2 bag play "$BAG_PATH" \
      --topics /gimbal/imu /gimbal/status /detector/armors \
      --clock \
      --loop \
      -r 1.0
    ;;

  # ===========================================================================
  # 闭环假云台
  # ===========================================================================
  fake)
    echo -e "${GREEN}=== 启动闭环假云台 ===${NC}"
    echo -e "${CYAN}订阅 /gimbal_cmd → 模拟云台跟踪 → 发布 GimbalStatus${NC}"
    python3 "$WORKSPACE/tools/fake_sensor_publisher.py"
    ;;

  # ===========================================================================
  # 启动 autoaim 节点
  # ===========================================================================
  run)
    echo -e "${GREEN}=== 启动 rm_autoaim ===${NC}"
    ros2 run rm_autoaim autoaim_node --ros-args \
      --params-file "$PARAMS_FILE"
    ;;

  # ===========================================================================
  # 编译 + 自动重启 autoaim
  # ===========================================================================
  rebuild)
    echo -e "${YELLOW}=== 编译 rm_autoaim ... ===${NC}"
    cd "$WORKSPACE"
    colcon build --packages-select rm_autoaim 2>&1 | tail -5

    if [ ${PIPESTATUS[0]} -eq 0 ]; then
      echo -e "${GREEN}编译成功! 正在重启 autoaim ...${NC}"
      source "$WORKSPACE/install/setup.bash"

      pkill -f "autoaim_node" 2>/dev/null
      sleep 0.5

      ros2 run rm_autoaim autoaim_node --ros-args \
        --params-file "$PARAMS_FILE"
    else
      echo -e "${RED}编译失败! 请检查错误。${NC}"
      exit 1
    fi
    ;;

  # ===========================================================================
  # 打开 PlotJuggler
  # ===========================================================================
  plot)
    echo -e "${GREEN}=== 启动 PlotJuggler ===${NC}"
    echo -e "${CYAN}操作: Streaming → Start ROS2 Topic Subscriber${NC}"
    echo -e "${CYAN}勾选: /autoaim/debug_world_points, /gimbal_cmd${NC}"
    ros2 run plotjuggler plotjuggler
    ;;

  # ===========================================================================
  # 打开 rqt_plot (轻量备选)
  # ===========================================================================
  rqt)
    echo -e "${GREEN}=== 启动 rqt_plot ===${NC}"
    rqt_plot /autoaim/debug_world_points/point/x \
             /autoaim/debug_world_points/point/y \
             /gimbal_cmd/target_yaw \
             /gimbal_cmd/fire_control
    ;;

  # ===========================================================================
  # 显示当前 UKF 参数
  # ===========================================================================
  show)
    echo -e "${CYAN}=== 当前 UKF 参数 (imm_ukf_tracker.hpp) ===${NC}"
    grep -A 20 "struct UKFParams" "$WORKSPACE/rm_autoaim/include/rm_autoaim/imm_ukf_tracker.hpp" | head -25
    echo ""
    echo -e "${CYAN}=== 当前 Aimer 参数 (aimer.hpp) ===${NC}"
    grep -A 25 "struct AimerParams" "$WORKSPACE/rm_autoaim/include/rm_autoaim/aimer.hpp" | head -30
    ;;

  # ===========================================================================
  # 离线分析当前 bag
  # ===========================================================================
  analyze)
    python3 "$WORKSPACE/tools/analyze_bag.py"
    ;;

  # ===========================================================================
  # 帮助
  # ===========================================================================
  *)
    echo ""
    echo -e "${CYAN}IMM-UKF 一键调参工具 (闭环版)${NC}"
    echo ""
    echo "用法: $0 <命令>"
    echo ""
    echo "命令:"
    echo "  play        循环播放 bag (仅 /detector/armors)"
    echo "  play-full   循环播放 bag (含 IMU/Status, 不用 fake)"
    echo "  fake        启动闭环假云台 (推荐)"
    echo "  run         启动 autoaim 节点"
    echo "  rebuild     编译 + 自动重启 autoaim"
    echo "  plot        打开 PlotJuggler"
    echo "  rqt         打开 rqt_plot"
    echo "  show        显示当前 UKF + Aimer 参数"
    echo "  analyze     离线分析 bag"
    echo ""
    echo -e "${YELLOW}闭环调参流程 (推荐):${NC}"
    echo "  终端1: $0 play"
    echo "  终端2: $0 fake"
    echo "  终端3: $0 run"
    echo "  终端4: $0 plot"
    echo "  改参数 → 终端3: Ctrl+C → $0 rebuild"
    echo ""
    echo -e "${YELLOW}调参口诀:${NC}"
    echo "  滞后 → 增 Q (q_vel, q_omega)"
    echo "  抖动 → 减 Q 或 增 R (r_pos)"
    echo "  不开火 → 检查 fire_tolerance, fire_min_frames"
    echo ""
    ;;
esac