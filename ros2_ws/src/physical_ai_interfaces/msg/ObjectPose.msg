# header.frame_id identifies the frame of pose; source_header records RGB-D.
std_msgs/Header header
std_msgs/Header source_header
string detection_id
string class_id
geometry_msgs/PoseWithCovariance pose
float32 confidence
float32 depth_m
float32 depth_mad_m
uint32 valid_depth_samples
bool orientation_valid

uint8 SOURCE_VISION=1
uint8 source_mode
