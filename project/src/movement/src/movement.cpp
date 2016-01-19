#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "movement.hpp"
#include "geometry_msgs/Twist.h"
#include <math.h>

Mover::Mover()
{
  //Variables
  keepMoving = true;
  gotTarget = false;
  nextId = 0;

  //Publishers
  commandPub = node.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);
  targetReachedRequest = node.advertise<std_msgs::Empty>("targetReached", 10);

  //Subscribers
  laserSub = node.subscribe("/scan", 1, &Mover::scanCallback, this);
  bumperSub = node.subscribe("mobile_base/events/bumperSub", 20, &Mover::bumperSubCallback, this);
//  imageSub = node.subscribe("/camera/rgb/image_rect_color", 10, &Mover::rgbCallback, this);
  getLocationSub = node.subscribe("locationTopic", 10, &Mover::getLocationCallback, this);
  targetFinishedSub = node.subscribe("targetFinishedTopic", 10, &Mover::targetFinishedCallback, this);
}

/*The movement.cpp includes following methodes:
 *
 * driveForwardOdom(double distance)[line xx]; forwards the robot to move a certain distance
 *
 * rotateOdom(double angle)[line xx]; forwards the robot to rotate a certain angle
 *
 * scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)[line xx]; checks in the Range of +/-30 deg if there is an obstacle in front of the robot
 *
 * bumperSubCallback(const kobuki_msgs::BumperEvent::ConstPtr& bumperSub_msg)[line xx]; reacts on pressed bumper of the robot
 *
 * getLocationCallback(const geometry_msgs::Vector3StampedConstPtr &vector)[line xx]; stores passed vector coordinates in member target
 *
 * targetFinishedCallback(const std_msgs::EmptyConstPtr empty)
 *
 * moveRandomly()[line xx]; if there is no aruco marker in the passed image the robot moves around randomly
 *
 * startMoving()[line xx]; Method which should be called externely. Includes the the while loop to keep node mover running
 */


void Mover::driveForwardOdom(double distance)
{
  //we will be sending commands of type "twist"
  geometry_msgs::Twist base_cmd;

  //the command will be to go forward at 0.2 m/s
  base_cmd.linear.y = base_cmd.angular.z = 0;
  base_cmd.linear.x = 0.2;

  ros::Rate rate(10.0);
  double distanceMoved = 0;
  ros::Time startTime = ros::Time::now();

  //while((t[s]*v[m/s] = travelled distance[m] < distance[m] )
  while (((ros::Time::now()-startTime).toSec()*0.2 < distance) && node.ok() && keepMoving)
    {
      //send the drive command
      commandPub.publish(base_cmd);
      rate.sleep();

      //distanceMoved = (ros::Time::now()-startTime).toSec()*0.2;
      //ROS_INFO("distance %f", distanceMoved);
      ros::spinOnce();
    }
}


void Mover::rotateOdom(double angle)
{
  //wait for the listener to get the first message
  listener.waitForTransform("base_footprint", "/odom", ros::Time(0), ros::Duration(1.0));

  //we will record transforms here
  tf::StampedTransform start_transform;
  tf::StampedTransform current_transform;

  //record the starting transform from the odometry to the base frame
  listener.lookupTransform("base_footprint", "/odom", ros::Time(0), start_transform);

  //we will be sending commands of type "twist"
  geometry_msgs::Twist base_cmd;
  //the command will be to rotate at 0.25 m/s
  if(angle<0)
    {
      base_cmd.linear.y = base_cmd.linear.x = 0;
      base_cmd.angular.z = 0.25;
    }
  else
    {
      base_cmd.linear.y = base_cmd.linear.x = 0;
      base_cmd.angular.z = -0.25;
    }
  ros::Rate rate(10.0);
  bool done = false;

  //get original angle in RAD
  tfScalar originAngle = tf::getYaw(start_transform.getRotation());
  ROS_INFO("original angle %f", originAngle);

  //bool needed to check if while loop has already passed first time
  bool runFirstTime = true;

  //Run loop until rotation has been done
  while (!done && node.ok())
    {
      tfScalar relativeAngle, currentAngle, previousAngle;

      //send the rotation command
      commandPub.publish(base_cmd);
      rate.sleep();

      if(runFirstTime==false){previousAngle = currentAngle;}

      //get the current transform
      try
      {
        listener.lookupTransform("base_footprint", "/odom", ros::Time(0), current_transform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
        break;
      }
      
      //get current Angle in RAD
      currentAngle = tf::getYaw(current_transform.getRotation());

      //see if there was a jump in the current Angle (to avoid problem with jump from -PI to +PI and vice versa)
      if(runFirstTime==false && fabs((currentAngle-previousAngle)) > 2.0)
        {
          currentAngle = (currentAngle + 2*M_PI);
          double difference = fabs((currentAngle-previousAngle));
          ROS_INFO("difference %f", difference);
        }

      //calculate already absolved angle
      relativeAngle = currentAngle - originAngle;
      relativeAngle = ((relativeAngle*180.0)/M_PI);
      runFirstTime = false;

      //check if rotation is done
      if(relativeAngle>angle)
        {
          done=true;
          keepMoving=true;
          runFirstTime =true;
        }
    }
}


//Callback of the /scan topic. Checks wether an obstacle disturbs forward movement
void Mover::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  int minIndex = ceil((MIN_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);
  int maxIndex = floor((MAX_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);

  float closestRange = scan->ranges[minIndex];

  for(int currIndex = minIndex+1; currIndex<=maxIndex; currIndex++)
    {
      if(scan->ranges[currIndex] < closestRange)
        {
          closestRange =	scan->ranges[currIndex];
        }
    }

  //ROS_INFO("closest range = %f", closestRange);
  if(closestRange<MIN_PROXIMITY_RANGE_M)
    {
      //check if you have a target we are heading for
      if(gotTarget==true)
        {
          std_msgs::Empty empty;
          targetReachedRequest.publish(empty); //send request if target is in front of us
        }

      ROS_INFO("Obstacle in %f!", closestRange);
      keepMoving=false;
      rotateOdom(30.0);
    }
  else
    {
      keepMoving = true;
    }
}

//Callback function for the bumperSub event. Stops the forward movement and initiates an rotation
void Mover::bumperSubCallback(const kobuki_msgs::BumperEvent::ConstPtr& bumperSub_msg)
{
  if(bumperSub_msg->state==1)
    {
      ROS_INFO("You just get bumped on %d!", bumperSub_msg->bumper);

      //Move backwards
      geometry_msgs::Twist base_cmd;
      base_cmd.linear.y = base_cmd.angular.z = 0;
      base_cmd.linear.x = -0.2;
      commandPub.publish(base_cmd);//drive backwards
      ros::Duration(0.5).sleep(); // sleep for half a second

      keepMoving=false;
      rotateOdom(45);
      ros::spinOnce();
    }
}
/*
void Mover::rgbCallback(const sensor_msgs::ImageConstPtr &msg)
{
  if(gotTarget)
    {

    }
}
*/

void Mover::getLocationCallback(const geometry_msgs::Vector3StampedConstPtr &vector)
{
  keepMoving = false;
  target.vector = vector->vector;
  gotTarget = true;
}

void Mover::targetFinishedCallback(const std_msgs::EmptyConstPtr empty)
{
  gotTarget = false;
  rotateOdom(180);
  driveForwardOdom(0.5);
  rotateOdom(358);
}

void Mover::moveRandomly()
{
  srand (time(NULL));
  double randDist = ((double) rand() / (RAND_MAX));
  double randAngle = ((double) 180*(rand() / (RAND_MAX)));

  ROS_INFO("Random Angle = %f", randAngle);
  rotateOdom(randAngle);
  ROS_INFO("Random Distance = %f", randDist);
  driveForwardOdom(randDist);

  ROS_INFO("Look around");
  rotateOdom(358);
  ROS_INFO("Found nothing");
}

//Method which will be called from run_stopper.cpp
void Mover::startMoving()
{
  ros::Rate rate(10);
  ROS_INFO("Start moving");

  while(ros::ok() && keepMoving)
    {
      if(gotTarget==false)
        {
          moveRandomly();
        }
      else
        {
          driveForwardOdom(0.75);
        }
      ros::spinOnce();
      rate.sleep();
    }
  while(!keepMoving)
    {

      rotateOdom(30);
      ros::spinOnce();
    }
}



