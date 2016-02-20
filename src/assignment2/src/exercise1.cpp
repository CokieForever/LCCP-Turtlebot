#include "turtlecommander.hpp"

turtleCommander::turtleCommander()
{
	m_readStartPosition=false;
	m_moveTurtle1Pub = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);
	m_subTurtle1Pose = n.subscribe("turtle1/pose", 1000, &turtleCommander::turtle1Posecallback, this);
}

void turtleCommander::turtle1Posecallback(turtlesim::Pose turtlePos)
{
	if(m_readStartPosition==false)
	{
		m_startPosition.x=turtlePos.x;
		m_startPosition.y=turtlePos.y;
		m_startPosition.theta=turtlePos.theta;
		m_readStartPosition=true;
		ROS_INFO("Start Position: x=%f y=%f Theta=%f", m_startPosition.x,m_startPosition.y,m_startPosition.theta);
	} 
	else
	{
		m_turtle1_pose.x=turtlePos.x;
		m_turtle1_pose.y=turtlePos.y;
		m_turtle1_pose.theta=turtlePos.theta;
		ROS_INFO_STREAM("x="<< m_turtle1_pose.x << "y=" << m_turtle1_pose.y<<"theta="<<m_turtle1_pose.theta);
		if(m_turtle1_pose.x<=0||m_turtle1_pose.x>=11.05||m_turtle1_pose.y<=0||m_turtle1_pose.y>=11.05)
		{ ROS_WARN("You hit the boundary!");}
	}
}

void turtleCommander::moveTurtleForward()
{
	geometry_msgs::Twist moveCommand;
	moveCommand.linear.x = 1.0;
	moveCommand.angular.z = 0;
	m_moveTurtle1Pub.publish(moveCommand);
	ros::spinOnce();
	ros::Duration time(1);
	time.sleep();
}

void turtleCommander::rotateTurtle()
{
	geometry_msgs::Twist moveCommand;
	moveCommand.linear.x = 0.0;
	moveCommand.angular.z = M_PI/4.0;
	m_moveTurtle1Pub.publish(moveCommand);
	ros::spinOnce();
	ros::Duration time(1);
	time.sleep();
}

bool turtleCommander::checkNodeState()
{
	bool state;
	state = n.ok();
	return state;
}
