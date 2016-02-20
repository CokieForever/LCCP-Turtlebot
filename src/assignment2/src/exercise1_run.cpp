#include "turtlecommander.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "turtle_controller");
  
  turtleCommander turtle;
  ros::Rate loop_rate(20);

  while (turtle.checkNodeState())
  {
    turtle.moveTurtleForward();
    turtle.rotateTurtle();
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}
