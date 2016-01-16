
#include "Stopper.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <stdlib.h>
#include "tf/transform_listener.h"
Stopper::Stopper()
{
  keepMoving = true;
  //Advertise a new publisher for the simulated robot's velocity command topic
  commandPub = node.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);
  //Subscribe to the simulated robot's laser scan topic
  laserSub = node.subscribe("/scan", 1, &Stopper::scanCallback, this);
}

//Send a velocity command
void Stopper::moveForward()	
{
  geometry_msgs::Twist msg;	//The default constructor will set all commands to 0
  msg.linear.x = FORWARD_SPEED_MPS;
  commandPub.publish(msg);
};

void Stopper::rotate()
{
    /* initialize random seed: */
    srand (time(NULL));
    geometry_msgs::Twist msg;
    ROS_INFO_STREAM("Rotation: "<<msg.angular.z);

 /*   listener.waitForTransform("base_footprint", "odom_combined", ros::Time(0), ros::Duration(1.0));


    //we will record transforms here
    tf::StampedTransform start_transform;
    tf::StampedTransform current_transform;

    //record the starting transform from the odometry to the base frame
    listener.lookupTransform("base_footprint", "odom_combined", ros::Time(0), start_transform);
*/
    //the command will be to go rotate at least until "rotation"
    float rotation = ((((float)rand()/(RAND_MAX))-1)*M_PI);
    msg.linear.x = 0.0;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 2.8;

    ros::Rate rate(10.0);
    commandPub.publish(msg);
    rate.sleep();

    //  bool done = false;
   /* while (!done && node.ok())
    {
      //send the drive command
      commandPub.publish(msg);
      rate.sleep();
      //get the current transform
      try
      {
        listener.lookupTransform("base_footprint", "odom_combined", ros::Time(0), current_transform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
        break;
      }

     //see how far we've traveled
     tf::Transform relative_transform = start_transform.inverse() * current_transform;
     double dist_moved = relative_transform.getRotation().getAngle();

     if(dist_moved > rotation) done = true;
   }*/
};

void Stopper::stepBack()
{
  geometry_msgs::Twist msg;	//The default constructor will set all commands to 0
  msg.linear.x = -0.3;
  commandPub.publish(msg);
};

//Process te incoming laser scan message
void Stopper::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	//Find the closest range between the defined minimum and maximum angles
	int minIndex = ceil((MIN_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);
	int maxIndex = floor((MAX_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);
	
	float closestRange = scan->ranges[minIndex];
	for(int currIndex = minIndex +1; currIndex<=maxIndex; currIndex++)
	{
		if (scan->ranges[currIndex] < closestRange)
		{
			closestRange = scan->ranges[currIndex];
		}
	}
	
	ROS_INFO_STREAM("Closest range: "<< closestRange);
	if(closestRange < MIN_PROXIMITY_RANGE_M)
	{
		ROS_INFO("Stop!");
		stepBack();
		ros::Duration(0.5).sleep();
		rotate();
		ros::Duration(1.5).sleep();
		moveForward();
	}
}

void Stopper::startMoving()
{
	ros::Rate rate(100);
	ROS_INFO("Start moving");
	
	//Keep spinning loop until user presses Ctrl+C or the robot got too close to an obstacle
	while(ros::ok() && keepMoving)
	{
		moveForward();
		ros::spinOnce(); //Need to call this function often to allow ROS to process incoming messages
		rate.sleep();
	}
}

