#include <ros/ros.h>
#include <turtlesim/Pose.h> 
#include <geometry_msgs/Twist.h> // For geometry_msgs::Twist
#include <stdlib.h> 

class turtleCommander
{
	public:
	
	turtleCommander();
	void moveTurtleForward();
	void rotateTurtle();
	bool checkNodeState();
	
	private:
	ros::NodeHandle n;
	ros::Publisher m_moveTurtle1Pub;
	ros::Subscriber m_subTurtle1Pose;
	turtlesim::Pose m_startPosition, m_turtle1_pose;
	void turtle1Posecallback(turtlesim::Pose turtlePos);
	bool m_readStartPosition = false;
};

	
