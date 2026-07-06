# FCU aggregated state (published by fcu_state_node)
std_msgs/Header header

# Connection
bool connected
string mode              # e.g. "GUIDED", "LOITER", "LAND"
bool armed
bool manual_input        # true if RC override active

# Battery
float32 voltage          # V
float32 remaining        # 0.0-1.0
float32 current          # A

# GPS
uint8 fix_type           # 0=NO_GPS, 1=NO_FIX, 2=2D, 3=3D, 4=DGPS, 5=RTK
uint8 satellites_visible

# Estimator (from /mavros/estimator_status)
bool attitude_status
bool vel_horiz_status
bool vel_vert_status
bool pos_horiz_status
bool estimator_ok        # true if all estimator flags are healthy

# Local position (republished from MAVROS for convenience)
float32 local_x
float32 local_y
float32 local_z
float32 roll
float32 pitch
float32 yaw
