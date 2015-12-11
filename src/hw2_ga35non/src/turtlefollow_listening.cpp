#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>

#include "utilities.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "turtlefollow_listening");
	ros::NodeHandle node;
	ROS_INFO("Initialized ROS.");
	
	std::vector<std::string> turtlesNames;
	for (int i=1 ; i < argc ; i++)
		turtlesNames.push_back(argv[i]);
	
	int nbTurtles = turtlesNames.size();
	if (nbTurtles == 0)
	{
		ROS_ERROR("The turtles names must be given as arguments.");
		return -1;
	}
	else if (nbTurtles == 1)
	{
		if (turtlesNames[0] == "turtle1")
		{
			ROS_ERROR("At least a leader and a follower must be given as arguments.");
			return -1;
		}
		turtlesNames.insert(turtlesNames.begin(), "turtle1");
		nbTurtles += 1;
	}

	ROS_INFO("The leader turtle is \"%s\"", turtlesNames[0].c_str());
	
	ros::Rate rate(10.0);
	std::vector<ros::Publisher> turtlesPublishers;
	for (std::vector<std::string>::iterator it = turtlesNames.begin()+1 ; it != turtlesNames.end(); it++)
	{
		ROS_INFO("Creating publisher for turtle \"%s\"...", it->c_str());
		ros::Publisher pub = node.advertise<geometry_msgs::Twist>("/"+(*it)+"/cmd_vel", 10);
		while (ros::ok() && pub.getNumSubscribers() <= 0)
			rate.sleep();
		checkRosOk(-1);
		turtlesPublishers.push_back(pub);
	}

	tf::TransformListener listener;
	ros::Duration(1.0).sleep();
	
	ROS_INFO("Starting listening.");
	while (ros::ok())
	{
		for (int i=1 ; i < nbTurtles ; i++)
		{
			tf::StampedTransform transform;
			try
			{
				listener.lookupTransform(turtlesNames[i], turtlesNames[i-1], ros::Time(0), transform);
			}
			catch (tf::TransformException& ex)
			{
				ROS_WARN("%s", ex.what());
				ros::Duration(1.0).sleep();
				continue;
			}

			geometry_msgs::Twist twist;
			twist.angular.z = 4.0 * atan2(transform.getOrigin().y(), transform.getOrigin().x());
			twist.linear.x = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) + pow(transform.getOrigin().y(), 2));
			turtlesPublishers[i-1].publish(twist);
		}
		rate.sleep();
	}

	return 0;
}

