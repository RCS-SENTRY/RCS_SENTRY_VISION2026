std_msgs/Header header

bool has_target
bool tracking
bool temp_lost
bool fire_ready

float32 target_distance
float32 yaw_error_deg
float32 pitch_error_deg

# aim_source:
# 0=none
# 1=tracker
# 2=pnp_fallback
# 3=ray_fallback
uint8 aim_source
string reason
