#include	<ros/ros.h>
#include	<tf/transform_listener.h>
#include	<turtlesim/Spawn.h>
#include	<geometry_msgs/Twist.h>

using namespace std;
std::string	turtle_name1, turtle_name2;

int	main(int	argc,	char**	argv)	{
        ros::init(argc,	argv,	"tf_listener");
        ros::NodeHandle	node;
        turtle_name1=argv[1];
        turtle_name2=argv[2];
        ros::service::waitForService("spawn");
        ros::ServiceClient	add_turtle	=	node.serviceClient<turtlesim::Spawn>("spawn");
        turtlesim::Spawn	srv;
	srv.request.name = turtle_name1;
        add_turtle.call(srv);
        ros::Publisher	turtle_vel	=	node.advertise<geometry_msgs::Twist>(turtle_name1+"/cmd_vel",	10);
        tf::TransformListener	listener;
        ros::Rate	rate(10.0);
        double time=0;

        while	(node.ok())	{
                      tf::StampedTransform	transform;
                      try	{
                              listener.waitForTransform(turtle_name1,	turtle_name2,	ros::Time(time), ros::Duration(10.0));
                              listener.lookupTransform(turtle_name1,	turtle_name2,	ros::Time(time),	transform);
                      }	catch	(tf::TransformException	ex)	{
                              ROS_ERROR("%s",ex.what());
                      }
                      geometry_msgs::Twist	vel_msg;
                      vel_msg.angular.z	=	4	*	atan2(transform.getOrigin().y(),transform.getOrigin().x());
                      vel_msg.linear.x	=	0.5	*	sqrt(pow(transform.getOrigin().x(),	2)	+  pow(transform.getOrigin().y(),	2));
                      turtle_vel.publish(vel_msg);
                      rate.sleep();
              }
              return	0;
      };
