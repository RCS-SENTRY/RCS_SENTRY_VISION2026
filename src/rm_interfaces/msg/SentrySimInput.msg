std_msgs/Header header

# ---- 战术/场地补充输入 ----
bool    enemy_in_view
float32 enemy_confidence
float32 enemy_distance_m
bool    on_supply
bool    on_fortress
bool    on_outpost
bool    on_highground
bool    can_claim_periodic_ammo
bool    posture_switch_requested

# ---- 可选状态覆盖 ----
bool    has_hp_override
int32   hp_override
bool    has_ammo_17_override
int32   ammo_17_override
bool    has_heat_override
int32   heat_override
bool    has_gold_override
int32   gold_override
bool    has_stage_remain_time_override
int32   stage_remain_time_override
bool    has_is_disengaged_override
bool    is_disengaged_override
bool    has_sentry_posture_override
uint8   sentry_posture_override
bool    has_can_activate_energy_mechanism_override
bool    can_activate_energy_mechanism_override
