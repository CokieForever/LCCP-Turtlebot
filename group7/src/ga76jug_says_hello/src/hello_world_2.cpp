#include <ros/ros.h>

int main(int argc, char **argv)
{
	ros::int(argc, argv, "hello");
	
	ros::NodeHandle nh;
	ros::Rate loop_rate(10);
	
	int count = 0;
	while(ros::ok())
	{
		ROS_INFO_STREAM("hello world" <<count);
		
		ros::spinOnce();
		loop_rate.sleep();
		count++;
	}

return 0;
}
