#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <sstream>

void poseCallback(const turtlesim::Pose::ConstPtr& msg)
{
	ROS_INFO("The turtle is at (%.2f, %.2f, %.2f, %.2f, %.2f)", msg->x, msg->y, msg->theta,
			msg->linear_velocity, msg->angular_velocity);
}

int main(int argc, char **argv)
{
	ros::init(argc,argv, "task1");
	ros::NodeHandle n;
	ros::Publisher command_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",1000);
	
	//subscribing to turtlesim pose
	ros::Subscriber pose_sub = n.subscribe<turtlesim::Pose>("/turtle1/pose",1000,poseCallback);
	ros::Rate loop_rate(0.5);

	while(ros::ok())
	{
		geometry_msgs::Twist vel_msg;
		vel_msg.linear.x = 1.0;
		vel_msg.linear.y = 0.0;
		vel_msg.linear.z = 0.0;
		vel_msg.angular.x = 0.0;
		vel_msg.angular.y = 0.0;
		vel_msg.angular.z = 0.0;

		command_pub.publish(vel_msg);
		ros::spinOnce();
		loop_rate.sleep();
	
		vel_msg.linear.x = 0.0;
		vel_msg.linear.y = 0.0;
		vel_msg.linear.z = 0.0;
		vel_msg.angular.x = 0.0;
		vel_msg.angular.y = 0.0;
		vel_msg.angular.z = 0.785;
		command_pub.publish(vel_msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
//	ros::spin();
	return 0;
}
