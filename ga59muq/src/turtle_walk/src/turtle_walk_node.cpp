#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"

//method to move the turtle 1m
void move_straight(double speed, double distance, bool goForw);
//method to rotate the turtle 45degrees
void rotation (double ang_speed, double angle, bool orientation);
//convert degrees to rad
double degrees_to_rad(double dec);
void goOnTillEdge(double speed, bool goForw);
void poseCallback (const turtlesim::Pose::ConstPtr & pose_msg);


const double x_end = 11.0;
const double y_end = 11.0;
const double x_start = 0.2;
const double y_start = 0.2;
double x_initial, x_final;
double y_initial, y_final;


ros::Publisher vel_publisher;
ros::Subscriber pose_subscriber;
turtlesim::Pose turtlesim_pose;

int main (int argc, char **argv)
{
  ros::init(argc, argv, "turtle_walk_node");
  ros::NodeHandle n;

  ros::Rate loop_rate(100);

  vel_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",100);
  pose_subscriber= n.subscribe("/turtle1/pose",100, poseCallback);
  bool goForw;


  if (turtlesim_pose.x>=x_end || turtlesim_pose.y >= y_end || turtlesim_pose.x<=x_start || turtlesim_pose.y <= y_start)
  {
    goForw=false;
  }
  else
  {
   goForw=true;
  }
  move_straight(2,1,goForw);
  double rotation_speed= 2;
  double rotation_angle= 0.7853982;
  rotation (rotation_speed, rotation_angle, false);
  goOnTillEdge (2, true);
  ROS_INFO("x final: %.2f, y final:%.2f", x_final, y_final);
  ros::spin();
  return 0;
}


void move_straight(double speed, double distance, bool goForw)
{
  geometry_msgs::Twist vel;
  double position=0;
  double t=0;


   if (goForw)
   {
      vel.linear.x=abs(speed);
   }
   else
   {
      vel.linear.x=-abs(speed);
   }

   vel.linear.y=0;
   vel.linear.z=0;
   vel.angular.x=0;
   vel.angular.y=0;
   vel.angular.z=0;

   double t_zero= ros::Time::now().toSec();
   ros::Rate loop_rate(100);
   x_initial= turtlesim_pose.x;
   y_initial=turtlesim_pose.y;
   ROS_INFO("x initial: %.2f, y initial:%.2f", x_initial, y_initial);
   do{
       vel_publisher.publish(vel);
       t=ros::Time::now().toSec();
       position=speed*(t-t_zero);
       ros::spinOnce();
       loop_rate.sleep();

     }while(position<distance);

   vel.linear.x=0;
   vel_publisher.publish(vel);


}

void rotation (double ang_speed, double angle, bool orientation)
{
  geometry_msgs::Twist vel;

  vel.linear.x=0;
  vel.linear.y=0;
  vel.linear.z=0;

  vel.angular.x=0;
  vel.angular.y=0;

  if (orientation)
    {
      vel.angular.z=-abs(ang_speed);
    }
  else
    {
      vel.angular.z=abs(ang_speed);
    }

  double ang_position=0;


  ros::Rate loop_rate(100);
  double t_zero= ros::Time::now().toSec();

  do{
      vel_publisher.publish(vel);
      double t=ros::Time::now().toSec();
      ang_position=ang_speed*(t-t_zero);
      ros::spinOnce();
      loop_rate.sleep();

    }while(ang_position<angle);

  vel.angular.z=0;
  vel_publisher.publish(vel);
}


double degrees_to_rad(double dec)
{
  double pi = 3.141593;
  double r= pi*dec/180;
  return r;
}


void goOnTillEdge(double speed, bool goForw)
{
  geometry_msgs::Twist vel;


   if (goForw)
   {
      vel.linear.x=abs(speed);
   }
   else
   {
      vel.linear.x=-abs(speed);
   }

   vel.linear.y=0;
   vel.linear.z=0;
   vel.angular.x=0;
   vel.angular.y=0;
   vel.angular.z=0;


   ros::Rate loop_rate(100);

   do{
       vel_publisher.publish(vel);
       ros::spinOnce();
       loop_rate.sleep();

     }while((turtlesim_pose.x<x_end && turtlesim_pose.x>x_start && turtlesim_pose.y<y_end && turtlesim_pose.y>y_start));
   ROS_WARN("CLOSE TO EDGE");
   x_final=turtlesim_pose.x;
   y_final=turtlesim_pose.y;
   vel.linear.x=0;
   vel_publisher.publish(vel);


}

void poseCallback(const turtlesim::Pose::ConstPtr &pose_msg)
{
  turtlesim_pose.x= pose_msg->x;
  turtlesim_pose.y= pose_msg->y;
  turtlesim_pose.theta= pose_msg->theta;
}
