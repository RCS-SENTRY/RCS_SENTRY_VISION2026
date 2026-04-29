// =============================================================================
// protocol.hpp — 底层通信协议定义
// =============================================================================
// 本文件定义了上位机与 STM32 下位机之间的串口通信帧结构。
// 结构体字段需与当前上下位机协议保持严格一致。
// =============================================================================
#ifndef RM_HW_BRIDGE__PROTOCOL_HPP_
#define RM_HW_BRIDGE__PROTOCOL_HPP_

#include <cstdint>
#include <cstddef>

#pragma pack(push, 1)

// =============================================================================
// 上位机 → 下位机：云台控制帧
// 对应 nuc.cpp Decode() 中的 'S' 'P' 帧
// =============================================================================
struct VisionToNucFrame
{
  uint8_t  head[2]          = {'S', 'P'};
  uint8_t  mode_TJ;         // 模式标志：0=无目标 1=有目标/控制信号
  float    yaw_TJ;          // 目标 yaw 角 (rad)
  float    yaw_vel_TJ;      // yaw 角速度 (rad/s)
  float    yaw_acc_TJ;      // yaw 角加速度 (rad/s²)
  float    pitch_TJ;        // 目标 pitch 角 (rad)
  float    pitch_vel_TJ;    // pitch 角速度 (rad/s)
  float    pitch_acc_TJ;    // pitch 角加速度 (rad/s²)
  int8_t   state_switch_TJ; // 姿态状态: 1=Move 2=Attack 3=Defense
  int8_t   fire_control_TJ; // 火控许可: 0=不发射 1=发射

  // 扩展控制/裁判动作负载
  uint8_t  protocol_version;
  uint8_t  goal_id;
  uint8_t  tactical_state;
  uint8_t  posture;
  uint8_t  fire_policy;
  uint8_t  spin_mode;
  uint8_t  supercap_mode;
  uint8_t  rule_action_type;
  uint16_t ammo_exchange_target_total;
  uint8_t  revive_cmd;
  uint8_t  remote_ammo_req_inc;
  uint8_t  remote_hp_req_inc;
  uint8_t  posture_cmd_referee;
  uint8_t  activate_energy_confirm;
  uint8_t  claim_periodic_ammo;
  uint16_t crc16_TJ;        // CRC16 校验
};

static_assert(
  sizeof(VisionToNucFrame) == 47,
  "VisionToNucFrame must match the current upper-to-lower protocol layout");

// =============================================================================
// 上位机 → 下位机：导航速度帧
// =============================================================================
struct NavToNucFrame
{
  uint8_t  head[4]   = {'N', 'A', 'V', 'I'};
  float    linear_x;        // 线速度 x (m/s)
  float    linear_y;        // 线速度 y (m/s)
  float    angular_z;       // 角速度 z (rad/s)
  int8_t   isReached;       // 导航状态: 0=未到达 1=已到达 -1=导航失败
  uint16_t crc16_TJ;        // CRC16 校验
};

// =============================================================================
// 下位机 → 上位机：状态反馈帧
// 对应 nuc.cpp Encode() 中的 'S' 'P' 帧
// =============================================================================
struct NucToVisionFrame
{
  uint8_t  head[2];                           // 帧头 'S' 'P'

  // 自瞄相关数据
  uint8_t  mode_TJ;                           // 0:空闲 1:自瞄 2:小符 3:大符
  float    yaw_TJ;
  float    yaw_vel_TJ;
  float    pitch_TJ;
  float    pitch_vel_TJ;
  float    roll_TJ;
  float    roll_vel_TJ;
  float    bullet_speed_TJ;
  uint16_t bullet_count_TJ;                   // 子弹累计发送次数
  int8_t   game_status_NAV;                   // 0:未开始 1:开始 2:结束

  // 底盘
  float    chassis_vx;
  float    chassis_vy;
  float    chassis_wz;

  // 赛事状况
  int32_t  game_progress;
  int32_t  stage_remain_time;
  int32_t  ally_outpost_hp;
  uint32_t event_data;

  // RobotStatus
  uint8_t  robot_id;
  uint8_t  robot_level;
  uint16_t current_HP;
  uint16_t maximum_HP;
  uint16_t shooter_barrel_cooling_value;
  uint16_t shooter_barrel_heat_limit;
  uint16_t chassis_power_limit;
  uint8_t  power_management_gimbal_output;
  uint8_t  power_management_chassis_output;
  uint8_t  power_management_shooter_output;

  // Sentry_info
  uint16_t exchanged_projectile_allowance;
  uint8_t  remote_exchange_projectile_count;
  uint8_t  remote_exchange_hp_count;
  uint8_t  can_confirm_revival;
  uint8_t  can_buy_instant_revival;
  uint16_t instant_revival_cost;
  uint8_t  is_disengaged;
  uint16_t team_17mm_exchange_remain;
  uint8_t  sentry_posture;
  uint8_t  can_activate_energy_mechanism;

  // Buff
  uint8_t  recovery_buff;
  uint16_t cooling_buff;
  uint8_t  defence_buff;
  uint8_t  vulnerability_buff;
  uint16_t attack_buff;
  uint8_t  remaining_energy;

  // hurt_data 受伤状况
  uint8_t  armor_id;
  uint8_t  HP_deduction_reason;

  // 允许发弹量
  uint16_t projectile_allowance_17mm;
  uint16_t projectile_allowance_42mm;
  uint16_t remaining_gold_coin;
  uint16_t projectile_allowance_fortress;

  // 实时发弹信息
  int16_t  supply_speed;

  // 底盘功率与超电数据
  float    chassis_power;
  float    remain_energy;
  uint16_t buffer_energy;

  // RFID
  uint32_t rfid_status;
  uint8_t  rfid_status_2;
  uint16_t shooter_17mm_1_barrel_heat;

  // 己方血量信息
  uint16_t ally_1_robot_HP;
  uint16_t ally_2_robot_HP;
  uint16_t ally_3_robot_HP;
  uint16_t ally_4_robot_HP;
  uint16_t reserved;
  uint16_t ally_7_robot_HP;
  uint16_t ally_outpost_HP;
  uint16_t ally_base_HP;

  uint16_t crc16_TJ;                          // CRC16 校验
};

static_assert(
  sizeof(NucToVisionFrame) == 145,
  "NucToVisionFrame must match the lower-machine TxPacket_TJ layout");

#pragma pack(pop)

// =============================================================================
// 帧头标识常量
// =============================================================================
constexpr uint8_t  FRAME_HEADER_VISION[2]  = {'S', 'P'};
constexpr uint8_t  FRAME_HEADER_NAV[4]     = {'N', 'A', 'V', 'I'};
constexpr size_t   VISION_TX_FRAME_SIZE    = sizeof(VisionToNucFrame);
constexpr size_t   NAV_TX_FRAME_SIZE       = sizeof(NavToNucFrame);
constexpr size_t   VISION_RX_FRAME_SIZE    = sizeof(NucToVisionFrame);

#endif  // RM_HW_BRIDGE__PROTOCOL_HPP_
