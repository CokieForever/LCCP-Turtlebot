#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc,argv,"talker");
  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
  ros::Rate loop_rate(0.5);

  int count=0;
  while(ros::ok())
  {
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 1.0;
    vel_msg.linear.y =  0;
    vel_msg.linear.z = 0;

    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = 0;

	
    
    ROS_INFO("[rw] linear.x=%.2f, angular.z=%.2f\n",vel_msg.linear.x, vel_msg.angular.z );

    chatter_pub.publish(vel_msg);
    ros::spinOnce();

    loop_rate.sleep();

    vel_msg.linear.x = 0;
    vel_msg.linear.y =  0;
    vel_msg.linear.z = 0;

    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = 1.0;
    chatter_pub.publish(vel_msg);
    ros::spinOnce();
    loop_rate.sleep();

    ++count;
  }
  return 0;
}
