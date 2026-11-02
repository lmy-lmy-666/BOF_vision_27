# Algorithm result. The actuator command is sent by aim_node over the gimbal link when configured.
std_msgs/Header header
builtin_interfaces/Time observation_stamp
bool valid
string status
uint64 track_id
int32 label
int32 plate
float64 yaw_rad
float64 pitch_rad
float64 flight_time_s
float64 observation_age_s
geometry_msgs/Point predicted_point_m
# Fire decision for this solution; fire_status explains a false value.
bool fire
string fire_status
