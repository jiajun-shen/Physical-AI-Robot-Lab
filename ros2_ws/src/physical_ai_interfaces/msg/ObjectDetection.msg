# Header stamp and frame identify the RGB measurement that produced this item.
std_msgs/Header header
string detection_id
string class_id
sensor_msgs/RegionOfInterest roi
uint32 image_width
uint32 image_height
float32 center_u_px
float32 center_v_px
uint32 mask_pixel_count
float32 confidence

uint8 SOURCE_VISION=1
uint8 source_mode
