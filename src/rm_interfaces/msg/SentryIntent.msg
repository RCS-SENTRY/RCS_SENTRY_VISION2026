std_msgs/Header header

uint8 protocol_version

# tactical_state:
# 0=IDLE
# 1=HOLD
# 2=ENGAGE
# 3=RETREAT
# 4=RESUPPLY
# 5=SEARCH
# 6=REPOSITION
# 7=RUNE_SUPPORT
uint8 tactical_state

# goal_id:
# 0=NONE
# 1=SAFE_HOLD
# 2=WAIT_REVIVE
# 3=SAFE_RETREAT_A
# 4=SAFE_RETREAT_B
# 5=SUPPLY_LEFT
# 6=SUPPLY_RIGHT
# 7=FORTRESS_HOLD
# 8=OUTPOST_HOLD
# 9=COMBAT_KITE_A
# 10=COMBAT_HOLD_A
# 11=MID_PRESSURE
# 12=HIGHGROUND_PEEK
# 13=COMBAT_PUSH_A
# 14=SEARCH_AREA_A
# 15=SEARCH_AREA_B
# 16=HIGHGROUND_SCAN
# 17=HIGHGROUND_CENTER
# 18=MID_CROSS
# 19=BASE_HOME
# 20=BASE_HOLD
uint8 goal_id

# posture_intent:
# 0=KEEP
# 1=MOVE
# 2=ATTACK
# 3=DEFENSE
uint8 posture_intent

# fire_policy:
# 0=HOLD_FIRE
# 1=CONSERVATIVE
# 2=NORMAL
# 3=AGGRESSIVE
uint8 fire_policy

# spin_mode:
# 0=OFF
# 1=ON
uint8 spin_mode

# supercap_mode:
# 0=OFF
# 1=KEEP
# 2=BURST
uint8 supercap_mode

# rule_action_type:
# 0=NONE
# 1=EXCHANGE_AMMO_AT_POINT
# 2=REMOTE_AMMO
# 3=REMOTE_HP
# 4=CONFIRM_REVIVE
# 5=INSTANT_REVIVE
# 6=CHANGE_POSTURE
# 7=ACTIVATE_ENERGY
uint8 rule_action_type

uint16 ammo_exchange_target_total
uint8 request_revive_confirm
uint8 request_instant_revive
uint8 request_remote_ammo_once
uint8 request_remote_hp_once
uint8 request_posture_referee
uint8 request_activate_energy

string reason
