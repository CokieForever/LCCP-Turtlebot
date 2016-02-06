#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <stdlib.h>
#include <stdio.h>
#include <sstream>


class MoveTurtle
{
public:
	MoveTurtle(char * name) 
	{
		sprintf(buffer_cmd_vel,"/%s/cmd_vel",name);
		sprintf(buffer_pose,"/%s/pose",name);
		vel_pub = n.advertise<geometry_msgs::Twist>(buffer_cmd_vel,100);
		pose_sub = n.subscribe(buffer_pose, 100, poseCallback);
		ROS_INFO("ECLIPSEInitial location: x=%.2f, y=%.2f",ts_pose.x, ts_pose.y);
	}
	void StartMoving()
	{
		move(15.0,5.0,0);
		rotate(1.0,(45*PI/180), 0);
	}
private:
	//attributes
	char buffer_cmd_vel[100];
	char buffer_pose[100];
	const double x_min = 0.0;
	const double y_min = 0.0;
	const double x_max = 11.0;
	const double y_max = 11.0;
	const double PI = 3.14156;
	ros::Publisher vel_pub;
	ros::Subscriber pose_sub;
	turtlesim::Pose ts_pose;
	
	//methods
	void poseCallback(turtlesim::Pose pose_msg)
	{
		ts_pose.x = pose_msg.x;
		ts_pose.y = pose_msg.y;
		ts_pose.theta = pose_msg.theta;
	}
	void rotate(double angular_speed, double angle, bool clockwise)
	{
		geometry_msgs::Twist vel_msg;
		vel_msg.linear.x=0;
		vel_msg.linear.y=0;
		vel_msg.linear.z=0;
		vel_msg.angular.x=0;
		vel_msg.angular.y=0;
		if (clockwise)
			vel_msg.angular.z=-abs(angular_speed);
		else
			vel_msg.angular.z=abs(angular_speed);
		double current_angle = 0.0;
		double t0 = ros::Time::now().toSec();
		ros::Rate loop_rate(10);
		do 
		{
			if(ts_pose.x < x_min || ts_pose.x > x_max || ts_pose.y < y_min || ts_pose.y > y_max)
			{
				//	ROS_INFO("Final location: x=%.2f, y=%.2f",ts_pose.x, ts_pose.y);
					break;
			}
			vel_pub.publish(vel_msg);
			double t1 = ros::Time::now().toSec();
			current_angle = angular_speed * (t1-t0);
			ros::spinOnce();
			loop_rate.sleep();
		} while (current_angle<angle);
		vel_msg.angular.z = 0;
		vel_pub.publish(vel_msg);
	}
	void move(double speed, double distance, bool isForward)
	{
			geometry_msgs::Twist vel_msg;
			if (isForward)
					vel_msg.linear.x = abs(speed);
			else
					vel_msg.linear.x = -abs(speed);
			vel_msg.linear.y=0;
			vel_msg.linear.z=0;
			vel_msg.angular.x=0;
			vel_msg.angular.y=0;
			vel_msg.angular.z=0;

			double t0 = ros::Time::now().toSec();
			double current_distance = 0;
			ros::Rate loop_rate(100);
	
			if(!(ts_pose.x < x_min || ts_pose.x > x_max || ts_pose.y < y_min || ts_pose.y > y_max))
			{
				do {
					if(ts_pose.x < x_min || ts_pose.x > x_max || ts_pose.y < y_min || ts_pose.y > y_max)
					{
							ROS_INFO("ECLIPSEFinal location: x=%.2f, y=%.2f",ts_pose.x, ts_pose.y);
							break;
					}
					vel_pub.publish(vel_msg);
					double t1 = ros::Time::now().toSec();
					current_distance = speed * (t1-t0);
					ros::spinOnce();
					loop_rate.sleep();
				} while((current_distance < distance));
			}
			vel_msg.linear.x=0;
			vel_pub.publish(vel_msg);
	}
	
}






int main(int argc, char **argv)
{
		ros::init(argc, argv, "ga73kec_a1_t1_node");
		ros::NodeHandle n;
		MoveTurtle turtle("turtle1");
		ros::Rate loop_rate(100);
		while(ros::ok())
		{
			turlte.StartMoving();
			ros::spinOnce();
			loop_rate.sleep();
		}//while ((ts_pose.x < x_max && ts_pose.x > x_min) &&
		//		(ts_pose.y < y_max && ts_pose.y > y_min));
		//ROS_INFO("I HIT THE WALL!");
		return 0;
}
