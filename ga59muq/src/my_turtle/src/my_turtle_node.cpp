#include <ros/ros.h>
#include <turtlesim/Spawn.h>
#include "geometry_msgs/Twist.h"
#include "std_srvs/Empty.h"
#include "turtlesim/Pose.h"
#include <string>

using namespace std;

ros::Publisher vel_publisher1;
ros::Publisher vel_publisher2;
ros::Subscriber pose_subscriber1;
ros::Subscriber pose_subscriber2;
turtlesim::Pose turtlesim_pose1;
turtlesim::Pose turtlesim_pose2;
void Rotation(double speed, double ang, double ang_speed);
double dec2rad(double dec);
void poseCallback1 (const turtlesim::Pose::ConstPtr & pose_msg);
void poseCallback2 (const turtlesim::Pose::ConstPtr & pose_msg);

int main(int argc, char **argv)
{
  //string robot_name = string(argv[1]);
  ros::init(argc, argv, "my_turtle");
  ros::NodeHandle n;

  ros::ServiceClient spawnClient=n.serviceClient<turtlesim::Spawn>("spawn");
  turtlesim::Spawn::Request req;
  turtlesim::Spawn::Response resp;

  req.x=2;
  req.y=3;
  req.theta=0;
  req.name="ga59muq";
  ros::service::waitForService("spawn");
  bool success = spawnClient.call(req ,resp) ;
  ros::Rate loop_rate(100);

  ROS_INFO_STREAM("spawned a turtle named" <<resp.name);



  vel_publisher1 = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",100);
  vel_publisher2 = n.advertise<geometry_msgs::Twist>("ga59muq/cmd_vel",100);
  pose_subscriber1= n.subscribe("turtle1/pose",100, poseCallback1);
  pose_subscriber2= n.subscribe("ga59muq/pose",100, poseCallback2);

  Rotation(2, dec2rad(360), 1.8);

  ros::service::waitForService ("clear");
  ros::param::set("background_b", 0);

  ros::ServiceClient clearClient=n.serviceClient<std_srvs::Empty>("/clear");
  std_srvs::Empty srv;
  clearClient.call(srv);
  ros::Duration(0.5).sleep();

 // ros::service::waitForService ("clear");
  ros::param::set("background_b", 255);
  clearClient.call(srv);

  ros::spin();
  return 0;

}


void Rotation(double speed, double ang, double ang_speed)
{
  geometry_msgs::Twist vel;
  double current_ang=0;
  double t=0;



   vel.linear.x=(speed);


   vel.angular.z=ang_speed;

   vel.linear.y=0;
   vel.linear.z=0;
   vel.angular.x=0;
   vel.angular.y=0;

   double t_zero= ros::Time::now().toSec();
   ros::Rate loop_rate(100);


  while(current_ang<ang){
       vel_publisher1.publish(vel);
       vel_publisher2.publish(vel);
       t=ros::Time::now().toSec();
       current_ang=ang_speed*(t-t_zero);
       ros::spinOnce();
       loop_rate.sleep();

     }

   vel.linear.x=0;
   vel.angular.z=0;
   vel_publisher1.publish(vel);
   vel_publisher2.publish(vel);


}

void poseCallback1(const turtlesim::Pose::ConstPtr &pose_msg )
{
  turtlesim_pose1.x= pose_msg->x;
  turtlesim_pose1.y= pose_msg->y;
  turtlesim_pose1.theta= pose_msg->theta;
}

void poseCallback2(const turtlesim::Pose::ConstPtr &pose_msg )
{
  turtlesim_pose2.x= pose_msg->x;
  turtlesim_pose2.y= pose_msg->y;
  turtlesim_pose2.theta= pose_msg->theta;
}

double dec2rad(double dec)
{
  return 3.14159265359*dec/180;
}
