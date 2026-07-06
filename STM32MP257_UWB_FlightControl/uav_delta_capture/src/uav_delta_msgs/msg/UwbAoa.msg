std_msgs/Header header

# UWB AOA measurements (after byte-swap and Kalman filter)
float32 distance_m          # distance in meters (raw cm / 100)
float32 azimuth_deg         # azimuth in degrees (signed: negative=left, positive=right)
float32 elevation_deg       # elevation in degrees (signed)

# Device IDs from frame
uint32 anchor_id
uint32 tag_id
uint16 tag_status

# Signal quality
float32 quality             # 0.0-1.0, based on data freshness
bool signal_valid            # false if no valid frame for >200ms
