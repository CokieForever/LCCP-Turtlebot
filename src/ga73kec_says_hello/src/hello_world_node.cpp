#include <ros/ros.h>

int main(int argc, char** argv)
{
	ros::init(argc,argv,"hello_world_node");
	ros::start();
	ROS_INFO_STREAM("Hello, world!");
	
	// ROS callbacks until receiving ctrl-c
	ros::spin();
	ros::shutdown();
	return 0;
}
