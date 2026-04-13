# =============================================================================
# GimbalCmd.msg — 上位机 → 下位机 云台控制命令
# =============================================================================
# 上层节点（自瞄/火控/决策）发布到此消息，rm_hw_bridge 纯转发。
# =============================================================================

# 目标角度（rad）
float64 target_yaw
float64 target_pitch

# 角速度（rad/s）
float64 yaw_vel
float64 pitch_vel

# 姿态状态: 0=默认 1=Move 2=Attack 3=Defense
int8    state_switch

# 火控许可: 0=不发射 1=发射
int8    fire_control

# 模式: 0=idle 1=auto_aim 2=small_buff 3=big_buff 4=outpost
uint8   mode

# 协议/决策扩展
uint8   protocol_version
uint8   goal_id
uint8   tactical_state
uint8   posture
uint8   fire_policy
uint8   spin_mode
uint8   supercap_mode
uint8   rule_action_type

# 裁判动作扩展
uint16  ammo_exchange_target_total
uint8   revive_cmd
uint8   remote_ammo_req_inc
uint8   remote_hp_req_inc
uint8   posture_cmd_referee
uint8   activate_energy_confirm
uint8   claim_periodic_ammo
