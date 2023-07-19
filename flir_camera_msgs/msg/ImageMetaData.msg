#
# meta data messages for camera images
#
std_msgs/Header header

# time stamp produced by camera, interpretation depends on camera model
uint64 camera_time

# computed brightness, either 0...255 (valid) or -1 (invalid)
int16 brightness

# exposure time in microseconds
uint32 exposure_time

# max allowed exposure time in microseconds
uint32 max_exposure_time

# gain in db
float32 gain
