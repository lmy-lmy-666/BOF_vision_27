# State at observation time in a stationary, gravity-aligned reference frame.
std_msgs/Header header
bool valid
string status
uint64 track_id
int32 label
string model
uint32 plates
int32 matched_plate
# vehicle: x,vx,y,vy,z,vz,yaw,omega,r,dr,dz
# outpost: x,vx,y,vy,z,vz,yaw,omega,dz1,dz2; radius_m is fixed
float64[] state
float64 radius_m
string spin
