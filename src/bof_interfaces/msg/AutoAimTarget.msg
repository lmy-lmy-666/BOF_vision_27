# Auto aim -> decision, topic /auto_aim/target. Positions in header.frame_id (aim reference, z up).
std_msgs/Header header
# lost / detecting / tracking / temporarily_lost / ...
string tracking_state
bool valid
uint64 track_id
int32 label
# vehicle / outpost
string target_type
uint32 armor_num
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
float64 v_yaw
float64 yaw
# Horizontal bearing and slant distance from the reference origin to the target centre.
float64 bearing_rad
float64 distance_m
# unknown / counter_clockwise / clockwise / stopped (outpost only)
string spin
bool locked
bool aiming
bool firing
string fire_status
float64 bullet_speed_mps
