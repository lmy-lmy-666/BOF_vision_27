# A single current-frame observation, not a tracked or predicted target state.
uint32 candidate_index
int32 label
bool valid
string status
# Camera optical coordinates, metres. Consume only when valid is true.
geometry_msgs/Point position_m
# Rodrigues vector in radians: plate x-right/y-down/z-normal -> camera optical.
geometry_msgs/Vector3 rotation_vector
float64 distance_m
float64 reprojection_error_px
float64 alternative_error_px
bool ambiguous
string geometry
string geometry_source
float64 width_m
float64 height_m
float64 image_ratio
