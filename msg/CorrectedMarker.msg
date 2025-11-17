# Custom message for corrected marker
std_msgs/Header header
uint32 id
geometry_msgs/PoseStamped original_pose
geometry_msgs/PoseStamped corrected_pose
float64 confidence
string correction_method
