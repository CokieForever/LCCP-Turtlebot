#define checkRosOk_v()	if (!ros::ok()) {ROS_ERROR("ROS interrupted."); return;}
#define checkRosOk(r)	if (!ros::ok()) {ROS_ERROR("ROS interrupted."); return r;}
