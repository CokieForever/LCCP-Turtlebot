#include <ros/ros.h>
#include <turtlesim/Spawn.h> 
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>
#include <stdlib.h>
#include <turtlesim/Pose.h>

class turtleConstructor
{
	public:
	
		turtleConstructor(std::string name);
		bool createTurtle();
		std::string getName();
		bool checkPose();
	
	private:
		ros::NodeHandle n;
		ros::Subscriber poseSubscribe;
		bool m_turtleCreated;
		void poseCallback(turtlesim::Pose turtlepose);
		std::string m_name;
		ros::ServiceClient newTurtle;
		ros::ServiceClient clear;
		turtlesim::Pose m_turtlepose;
};
		
