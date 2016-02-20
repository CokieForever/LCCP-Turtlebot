#include <ros/ros.h>
#include <turtlesim/Spawn.h> 
#include <std_srvs/Empty.h>
#include <stdlib.h>

class turtleConstructor
{
	public:
	
		turtleConstructor(std::string name);
		std::string getName();
	
	private:
		ros::NodeHandle n;
		bool m_turtleCreated;
		bool createTurtle();
		std::string m_name;
		ros::ServiceClient newTurtle;
		ros::ServiceClient clear;
};
		
