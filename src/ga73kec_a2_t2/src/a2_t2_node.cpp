#include "ros/ros.h"
#include "turtlesim/Spawn.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <string>
#include <cstdlib>

turtlesim::Pose turtle1_pose, ga73kec_pose;


void turtle1CallBack(turtlesim::Pose t1_position)
{
  turtle1_pose.x = t1_position.x;
  turtle1_pose.y = t1_position.y;
  turtle1_pose.theta = t1_position.theta;
}

void ga73kecCallBack(turtlesim::Pose ga_position)
{
  ga73kec_pose.x = ga_position.x;
  ga73kec_pose.y = ga_position.y;
  ga73kec_pose.theta = ga_position.theta;
  //ROS_INFO("(%.6f,%.6f,%.6f)",ga73kec_pose.x,ga73kec_pose.y,ga73kec_pose.theta);
}

int main(int argc, char**argv)
{

  ros::init(argc,argv,"a2_t2_node");
  ros::NodeHandle nh;

  ros::ServiceClient client = nh.serviceClient<turtlesim::Spawn>("spawn");
  ros::ServiceClient clear = nh.serviceClient<std_srvs::Empty>("clear");
  turtlesim::Spawn srv;
  std_srvs::Empty clear_srv;
  srv.request.name = "ga73kec";
  srv.request.x = 2;
  srv.request.y = 4;
  srv.request.theta = 0;
  bool bSrvCall = client.call(srv);
  int count = 0;


  //get current position of the turtles
  ros::Subscriber turtle1_sub = nh.subscribe<turtlesim::Pose>("turtle1/pose",1000, &turtle1CallBack);
  ros::Subscriber ga73kec_sub = nh.subscribe<turtlesim::Pose>("ga73kec/pose",1000, &ga73kecCallBack);
  if (bSrvCall)
  {
    ROS_INFO("Spawned new turtle named ga73kec at the coodinates (%.2f,%.2f,%.2f)", ga73kec_pose.x,ga73kec_pose.y,ga73kec_pose.theta);

  }
  ros::Publisher turtle1_pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);
  ros::Publisher ga73kec_pub = nh.advertise<geometry_msgs::Twist>("ga73kec/cmd_vel", 1000);


  geometry_msgs::Twist turtle1_msg, ga73kec_msg;

  turtle1_msg.linear.x = 2;
  turtle1_msg.linear.y = 0;
  turtle1_msg.linear.z = 0;
  turtle1_msg.angular.x = 0;
  turtle1_msg.angular.y = 0;
  turtle1_msg.angular.z = 1.8;
  //same for the other turtle
  ga73kec_msg.linear.x = 2;
  ga73kec_msg.linear.y = 0;
  ga73kec_msg.linear.z = 0;
  ga73kec_msg.angular.x = 0;
  ga73kec_msg.angular.y = 0;
  ga73kec_msg.angular.z = 1.8;

  double t0 = ros::Time::now().toSec();
  double t1, theta;
  ros::Rate loop_rate(100);
  while(ros::ok())
  { 

    t1 = ros::Time::now().toSec();
    theta = (t1-t0) * 1.8;

    turtle1_pub.publish(turtle1_msg);
    ga73kec_pub.publish(ga73kec_msg);
    ros::spinOnce();
    loop_rate.sleep();

    ROS_INFO("from callback (%.2f,%.2f,%.2f) theta: %.2f",ga73kec_pose.x, ga73kec_pose.y, ga73kec_pose.theta, theta);

    //set background if one resolution is done
    //if (ga73kec_pose.x < 2 && ga73kec_pose.y < 4 && count && ga73kec_pose.x && ga73kec_pose.y)
    if (theta > 6.3)
    {

      break;
    }
    count++;
   //ROS_INFO("from callback (%.2f,%.2f,%.2f)",ga73kec_pose.x, ga73kec_pose.y, ga73kec_pose.theta);
  }

  //stop the turtles
  turtle1_msg.linear.x = 0;
  turtle1_msg.linear.y = 0;
  turtle1_msg.linear.z = 0;
  turtle1_msg.angular.x = 0;
  turtle1_msg.angular.y = 0;
  turtle1_msg.angular.z = 0;
  ga73kec_msg.linear.x = 0;
  ga73kec_msg.linear.y = 0;
  ga73kec_msg.linear.z = 0;
  ga73kec_msg.angular.x = 0;
  ga73kec_msg.angular.y = 0;
  ga73kec_msg.angular.z = 0;
  turtle1_pub.publish(turtle1_msg);
  ga73kec_pub.publish(ga73kec_msg);



  nh.setParam("/background_b",0);
  clear.call(clear_srv);
  ros::Duration(0.5).sleep();
  nh.setParam("/background_b",255);
  clear.call(clear_srv);
  ros::Duration(0.5).sleep();

  ros::spin();
  return 0;
}
