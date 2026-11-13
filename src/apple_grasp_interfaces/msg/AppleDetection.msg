# Stable 3D apple detection expressed in the robot base frame.
std_msgs/Header header
bool detected
geometry_msgs/Point center
geometry_msgs/Point grasp_point
float32 radius
float32 confidence
uint32 point_count
float32 fit_residual
string failure_reason

