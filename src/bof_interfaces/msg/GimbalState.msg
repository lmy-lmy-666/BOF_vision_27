# Gimbal link feedback published by aim_node (serial mode), for recording and replay.
# Stamp is host receive time. Angles in radians with pitch up positive (converted from the link).
std_msgs/Header header
uint8 mode
float64 yaw
float64 yaw_vel
float64 pitch
float64 pitch_vel
float64 bullet_speed_mps
uint32 bullet_count
