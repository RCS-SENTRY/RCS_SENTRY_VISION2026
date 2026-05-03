std_msgs/Header header

uint64 frame_index
uint64 bt_tick_index

uint8  reported_posture
uint8  current_posture
uint8  preferred_posture
uint8  desired_posture
uint8  pending_posture_target
bool   posture_switch_pending
uint32 posture_cooldown_remaining_ms
uint32 posture_debuff_threshold_ms
uint32 current_posture_remaining_before_debuff_ms
uint32 preferred_posture_remaining_before_debuff_ms
bool   current_posture_debuffed
bool   preferred_posture_debuffed
uint32 attack_posture_accumulated_ms
uint32 defense_posture_accumulated_ms
uint32 move_posture_accumulated_ms
bool   attack_posture_debuffed
bool   defense_posture_debuffed
bool   move_posture_debuffed

uint8  tactical_state
uint8  rule_action_type
string desired_goal
uint8  fire_policy
uint8  spin_mode
uint8  supercap_mode

int32  hp
int32  ammo_17
int32  heat
bool   enemy_in_view
bool   referee_link_fresh
bool   sim_input_fresh
uint32 referee_status_age_ms
uint32 sim_input_age_ms

uint8  revive_cmd
uint8  remote_ammo_req_inc
uint8  remote_hp_req_inc
uint8  posture_cmd_referee
uint8  activate_energy_confirm
uint8  claim_periodic_ammo

string tactical_reason
string rule_reason
string goal_reason
string posture_reason
string spin_reason
string executor_summary
string last_rule_command
string input_health_reason
string last_nav_command
string last_shooter_command
