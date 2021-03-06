#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <turtlesim/Spawn.h>
#include <geometry_msgs/Twist.h>

std::string turtle_name, leader_name;

int main(int argc, char** argv){
	ros::init(argc, argv, "turtle5_tf_listener");
	turtle_name = argv[1];
  	leader_name = argv[2];
	
	ros::NodeHandle node;

	ros::service::waitForService("spawn");
	ros::ServiceClient add_turtle = node.serviceClient<turtlesim::Spawn>("spawn");
	turtlesim::Spawn srv;
	srv.request.name = turtle_name;
	srv.request.x = 2;
	srv.request.y = 2;

	add_turtle.call(srv);

	ros::Publisher turtle_vel =  node.advertise<geometry_msgs::Twist>(turtle_name + "/cmd_vel", 10);

	tf::TransformListener listener;
	ros::Rate rate(10.0);

	while(node.ok()){
		
		tf::StampedTransform transform;
		try{
			listener.lookupTransform(turtle_name, leader_name, ros::Time(0), transform);
		   }
		catch(tf::TransformException ex){ROS_ERROR("%s",ex.what());}

		geometry_msgs::Twist vel_msg;
		vel_msg.angular.z = 4*atan2(transform.getOrigin().y(), transform.getOrigin().x());
		vel_msg.linear.x = 0.5*sqrt(pow(transform.getOrigin().x(), 2) + pow(transform.getOrigin().y(), 2));
		turtle_vel.publish(vel_msg);

		rate.sleep();
	}
	return 0;
}

