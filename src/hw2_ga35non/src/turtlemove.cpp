#include "ros/ros.h"
/*#include "std_msgs/String.h"*/

#include <sstream>
#include <math.h>
#include <geometry_msgs/Twist.h>


int main(int argc, char **argv)
{
	ros::init(argc, argv, "turtlemove");
	ROS_INFO("Initialized ROS.");

	ROS_INFO("Publishing topic...");
	ros::NodeHandle nodeHandle;
	ros::Publisher turtlesimPublisher = nodeHandle.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
	
	ROS_INFO("Waiting for subscribers...");
	ros::Rate loopRate(10);
	int numSubscribers = 0;
	while ((numSubscribers = turtlesimPublisher.getNumSubscribers()) <= 0)
		loopRate.sleep();
	
	ROS_INFO("%d subscriber%s. Sending message...", numSubscribers, numSubscribers > 1 ? "s" : "");
	geometry_msgs::Twist twist;
	memset(&twist, 0, sizeof(geometry_msgs::Twist));
	twist.angular.z = M_PI / 4.0;
	twist.linear.x = 0.0;
	turtlesimPublisher.publish(twist);

	ROS_INFO("Message sent, waiting 3 seconds...");
	ros::Duration(3).sleep();

	ROS_INFO("Bye!");
	return 0;
}
