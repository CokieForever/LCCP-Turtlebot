#include "exercise2lib.hpp"

turtleConstructor::turtleConstructor(std::string turtlename)
{
	m_name=turtlename;
	m_turtleCreated=false;
	newTurtle = n.serviceClient<turtlesim::Spawn>("spawn");

	std::string poseName;
	poseName = turtlename;
	char pose[] = "/pose";
	for(int i=0;i<5;i++)
	{
				poseName.push_back(pose[i]);
	}
	poseSubscribe = n.subscribe(poseName, 1000, &turtleConstructor::poseCallback, this);
}

bool turtleConstructor::createTurtle()
{

  turtlesim::Spawn srv;
  srv.request.x = 2;
  srv.request.y = 4;
  srv.request.theta = 0;
  srv.request.name = m_name;
  newTurtle.waitForExistence();
  newTurtle.call(srv);
}

std::string turtleConstructor::getName()
{
  std::string buffer;
  buffer = m_name;
  return buffer;
}

void turtleConstructor::poseCallback(turtlesim::Pose turtlePose)
{
    m_turtlepose.x = turtlePose.x;
    m_turtlepose.y = turtlePose.y;
    m_turtlepose.theta = turtlePose.theta;
}

bool turtleConstructor::checkPose()
{
	if(((m_turtlepose.x<2.01)&&(m_turtlepose.x>1.99))&&m_turtlepose.y<4.01)
	{ROS_INFO_STREAM("cycle completed");return true;}
	else{return false;}
}
