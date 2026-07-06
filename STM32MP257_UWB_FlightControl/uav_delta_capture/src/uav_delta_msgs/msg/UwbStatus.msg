std_msgs/Header header

# UWB position in room frame (meters)
float32 x
float32 y
float32 z

# Quality
uint8 anchor_count
float32 quality

# Signal health
bool signal_valid
float32 age_sec

# Navigation state
string nav_state
float32 distance_to_target
