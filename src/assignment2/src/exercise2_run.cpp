#include "turtlecommander.hpp"
#include "exercise2lib.hpp"

int main(int argc, char **argv)
{
  std::string turtlename = argv[1];
  ros::init(argc, argv, "turtle_mover");
  
  turtleConstructor turtle(turtlename);
  turtle.createTurtle();
  std::string velName, poseName;
  velName = turtlename;
  char vel[] = "/cmd_vel";
  for(int i=0;i<8;i++)
  {
    velName.push_back(vel[i]);
  }

  ros::NodeHandle node;
  ros::Publisher moveTurtle1Pub = node.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);
  ros::Publisher moveTurtle2Pub = node.advertise<geometry_msgs::Twist>(velName, 1000);
  ros::ServiceClient clear = node.serviceClient<std_srvs::Empty>("reset");
  std_srvs::Empty empty;
  geometry_msgs::Twist moveGa76jug;
  moveGa76jug.linear.x = 2;
  moveGa76jug.angular.z = 1.8;
  
  geometry_msgs::Twist moveTurtle1;
  moveTurtle1.linear.x = 2;
  moveTurtle1.angular.z = 1.8;


  ros::Rate loop_rate(100); /* set loopRate to 100 so that the spinOnce() and therefore the Callback function will 
							   be called more once in 2 seconds and the achievment of the startpos is noticed early
							   enough */
  int count = 100;							   
  while (ros::ok())
  {


		if((turtle.checkPose() == true) && (count <=0))
		{
			moveGa76jug.linear.x = 0;
			moveGa76jug.angular.z = 0;
			moveTurtle1.linear.x = 0;
			moveTurtle1.angular.z = 0;

			node.setParam("background_b", 0);
			clear.call(empty);
			node.setParam("background_b", 0);
			clear.call(empty);

		}
		else{count --;}

		moveTurtle1Pub.publish(moveTurtle1);
		moveTurtle2Pub.publish(moveGa76jug);
		ros::spinOnce();
		loop_rate.sleep();

			}
		return 0;
	}



