# =============================================================================
# GimbalStatus.msg — 下位机 → 上位机 全状态反馈
# =============================================================================
# rm_hw_bridge 从串口解析 NucToVisionFrame 后发布此消息。
# =============================================================================

std_msgs/Header header

# ---- 云台 ----
float32 gimbal_yaw          # deg (CCW+, 逆时针增大, 与 ROS REP-103 一致)
float32 gimbal_pitch        # deg (抬头为负, 正=低头)
float32 gimbal_roll         # deg
float32 gimbal_yaw_rate     # deg/s
float32 gimbal_pitch_rate   # deg/s
float32 gimbal_roll_rate    # deg/s
float32 bullet_speed        # m/s
uint16  bullet_count        # 子弹累计发送次数
uint8   mode                # 当前模式
int8    game_status_nav     # 0=未开始 1=开始 2=结束

# ---- 底盘 ----
float32 chassis_vx
float32 chassis_vy
float32 chassis_wz
float32 chassis_power
float32 remain_energy
int16   supply_speed

# ---- 裁判系统 ----
int32   game_progress
int32   stage_remain_time
uint32  event_data
uint8   robot_id
uint8   robot_level
uint16  current_hp
uint16  maximum_hp
uint16  shooter_barrel_cooling_value
uint16  shooter_barrel_heat_limit
uint16  chassis_power_limit
uint8   power_management_gimbal_output
uint8   power_management_chassis_output
uint8   power_management_shooter_output

# ---- 哨兵信息 ----
uint16  exchanged_projectile_allowance
uint8   remote_exchange_projectile_count
uint8   remote_exchange_hp_count
uint8   can_confirm_revival
uint8   can_buy_instant_revival
uint16  instant_revival_cost
uint8   is_disengaged
uint16  team_17mm_exchange_remain
uint8   sentry_posture
uint8   can_activate_energy_mechanism

# ---- Buff / Hurt ----
uint8   recovery_buff
uint16  cooling_buff
uint8   defence_buff
uint8   vulnerability_buff
uint16  attack_buff
uint8   remaining_energy
uint8   armor_id
uint8   hp_deduction_reason

# ---- 发弹 / 金币 ----
uint16  projectile_allowance_17mm
uint16  projectile_allowance_42mm
uint16  remaining_gold_coin
uint16  projectile_allowance_fortress

# ---- 功率 / RFID / 热量 ----
uint16  buffer_energy
uint32  rfid_status
uint8   rfid_status_2
uint16  shooter_17mm_1_barrel_heat

# ---- 己方血量信息 ----
uint16  ally_1_robot_hp
uint16  ally_2_robot_hp
uint16  ally_3_robot_hp
uint16  ally_4_robot_hp
uint16  ally_reserved_hp
uint16  ally_7_robot_hp
uint16  ally_outpost_hp
uint16  ally_base_hp
