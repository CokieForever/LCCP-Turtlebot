// This program publishes velocity
// messages for turtlesim.
#include <ros/ros.h>
#include <turtlesim/Pose.h> 
#include <geometry_msgs/Twist.h> // For geometry_msgs::Twist
#include <stdlib.h> // For rand () and RAND_MAX
#include <turtlesim/Spawn.h> 
#include <std_srvs/Empty.h>
turtlesim::Pose turtle1_pose, ga76jug_pose;

void ga76jugPosecallback(turtlesim::Pose tempTurtle)
{
	ga76jug_pose.x=tempTurtle.x;
	ga76jug_pose.y=tempTurtle.y;
	ga76jug_pose.theta=tempTurtle.theta;
	//ROS_INFO_STREAM("x="<< ga76jug_pose.x << "y=" << ga76jug_pose.y<<"theta="<<ga76jug_pose.theta);

}

void turtle1Posecallback(turtlesim::Pose tempTurtle)
{
	turtle1_pose.x=tempTurtle.x;
	turtle1_pose.y=tempTurtle.y;
	turtle1_pose.theta=tempTurtle.theta;
   // ROS_INFO_STREAM("x="<< turtle1_pose.x << "y=" << turtle1_pose.y<<"theta="<<turtle1_pose.theta);
}

bool checkPose(void)
{
	if(((ga76jug_pose.x<2.01)&&(ga76jug_pose.x>1.99))&&ga76jug_pose.y<4.01)
	{ROS_INFO_STREAM("checked position true");return true;}
	else
	{ROS_INFO_STREAM("checked position false"<<ga76jug_pose.x<<ga76jug_pose.y);return false;}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "turtle_mover");
  ros::NodeHandle n;

  ros::Publisher moveTurtle1Pub = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);
  ros::Publisher moveGa76jugPub = n.advertise<geometry_msgs::Twist>("ga76jug/cmd_vel", 1000);
  ros::Subscriber subGa76jugPose = n.subscribe("ga76jug/pose", 1000, ga76jugPosecallback);
  ros::Subscriber subTurtle1Pose = n.subscribe("turtle1/pose", 1000, turtle1Posecallback);

  ros::ServiceClient newTurtle = n.serviceClient<turtlesim::Spawn>("spawn");
  ros::ServiceClient clear = n.serviceClient<std_srvs::Empty>("reset");
  std_srvs::Empty empty;
  turtlesim::Spawn srv;
  
  srv.request.x = 2;
  srv.request.y = 4;
  srv.request.theta = 0;
  srv.request.name = "ga76jug";
  newTurtle.waitForExistence();
  newTurtle.call(srv);
  
  srand(time(0));
  
  geometry_msgs::Twist moveGa76jug;
  moveGa76jug.linear.x = 2;
  moveGa76jug.linear.y = 0;
  moveGa76jug.linear.z = 0;

  moveGa76jug.angular.x = 0;
  moveGa76jug.angular.y = 0;
  moveGa76jug.angular.z = 1.8;
  
  geometry_msgs::Twist moveTurtle1;
  moveTurtle1.linear.x = 2;
  moveTurtle1.linear.y = 0;
  moveTurtle1.linear.z = 0;

  moveTurtle1.angular.x = 0;
  moveTurtle1.angular.y = 0;
  moveTurtle1.angular.z = 1.8;


  ros::Rate loop_rate(100); /* set loopRate to 100 so that the spinOnce() and therefore the Callback function will 
							   be called more once in 2 seconds and the achievment of the startpos is noticed early
							   enough */
  int count = 100;							   
  while (ros::ok())
  {


	if((checkPose() == true) && (count <=0))
	{
		moveGa76jug.linear.x = 0;
		moveGa76jug.angular.z = 0;
  		moveTurtle1.linear.x = 0;
		moveTurtle1.angular.z = 0;
	
		n.setParam("background_b", 0);
		clear.call(empty);
		n.setParam("background_b", 0);
		clear.call(empty);
		
	}
	else{count --;}
	moveTurtle1Pub.publish(moveTurtle1);
    moveGa76jugPub.publish(moveGa76jug);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
