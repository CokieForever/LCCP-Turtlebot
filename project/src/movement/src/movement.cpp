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
  reachedTarget = false;
  nextId = 0;
  m_searchMarker = 0;

  //distance between marker and turtlebot
  m_distanceToMarker = 9999;
  //Publishers
  commandPub = node.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);
  targetReachedRequest = node.advertise<std_msgs::Empty>("targetReached", 10);

  //Subscribers
  laserSub = node.subscribe("/scan", 1, &Mover::scanCallback, this);
  bumperSub = node.subscribe("mobile_base/events/bumperSub", 20, &Mover::bumperSubCallback, this);
//  imageSub = node.subscribe("/camera/rgb/image_rect_color", 10, &Mover::rgbCallback, this);
  getLocationSub = node.subscribe("/markerinfo", 10, &Mover::getLocationCallback, this);
  targetFinishedSub = node.subscribe("targetFinishedTopic", 10, &Mover::targetFinishedCallback, this);
}


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

  bool done = false;

  //get original angle in RAD
  tfScalar originAngle = tf::getYaw(start_transform.getRotation());
  ROS_INFO("original angle %f", originAngle);

  //bool needed to check if while loop has already passed first time
  bool runFirstTime = true;

  ros::Rate rate2(10);
  //Run loop until rotation has been done
  while (!done && node.ok() && !gotTarget)
    {
      tfScalar relativeAngle, currentAngle, previousAngle;

      //send the rotation command
      commandPub.publish(base_cmd);
      rate2.sleep();

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
      ros::spinOnce();

    }
  ros::spinOnce();
}


//Callback of the /scan topic. Checks wether an obstacle disturbs forward movement
void Mover::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  int minIndex = ceil((MIN_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);
  int maxIndex = floor((MAX_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);

  float closestRange = scan->ranges[minIndex];
  //save distance between robot and marker (which is in the center)
  m_distanceToMarker = scan->ranges[312];
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
      keepMoving=false;
      //check if you have a target we are heading for
      if(gotTarget==true)
        {
          gotTarget = false;
          reachedTarget = true;
          m_searchMarker++;
          ROS_INFO("Target reached!!! New Target Id: %d", m_searchMarker);
          rotateOdom(180);
          driveForwardOdom(0.75);
        }
       else
        {
          ROS_INFO("Obstacle in %f!", closestRange);

          rotateOdom(30.0);
        }
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

void Mover::approachMarker()
{
    ros::Rate rateX(50);
    geometry_msgs::Twist vel_msg;
    vel_msg.angular.z = m_angularVelocity;
    vel_msg.linear.x = 0.15;
    commandPub.publish(vel_msg);
    ROS_INFO("Distance to marker: %f", floor(m_distanceToMarker*10)/10 );
    if (floor(m_distanceToMarker*10)/10 == 0.7 && gotTarget)
    {
        if (m_searchMarker == 7)
        {
            ROS_INFO("Reached all targets!!!");
            vel_msg.angular.z = 0;
            vel_msg.linear.x = 0;
            commandPub.publish(vel_msg);
            rotateOdom(358);
        }
        ROS_INFO("Target Reached!!! The Marker is %d", m_searchMarker++);
        ROS_INFO("Searching next target: %d , distance is %f", m_searchMarker, m_distanceToMarker);

        //stop robot and start random search
        vel_msg.angular.z = 0;
        vel_msg.linear.x = 0;
        commandPub.publish(vel_msg);
        rotateOdom(180);
        //reset search
        gotTarget = false;
        reachedTarget = false;
    }
    rateX.sleep();
    ros::spinOnce();
}

void Mover::getLocationCallback(const detect_marker::MarkersInfos::ConstPtr &marker_msg)
{
   if (marker_msg->infos.size())
     {
      ROS_INFO("location callback!");
      //map the coordiante to the center
      int i_array_length = marker_msg->infos.size();
      float f_Xm = 0.0;
      for (int i=0; i<i_array_length; i++)
      {
        //check if marker id is equal to the next id that the bot is searching
        if (marker_msg->infos[i].id == m_searchMarker)
        {
            f_Xm = marker_msg->infos[i].x;
            //got a new target!
            gotTarget = true;
        }
      }
      ROS_INFO("Location of marker: %f", f_Xm);
      //was if (!reachedTarget && f_Xm!=0.0 && gotTarget)
      if (!reachedTarget && gotTarget)
      {
        //adjust robot, so the marker actually is in the center
        if (f_Xm < 0)
        {
          //rotate bot to the right
          m_angularVelocity = -f_Xm*0.8;
          ROS_INFO("Rotate to Left");
          approachMarker();
        }
        else if (f_Xm > 0)
        {
          //rotate bot to the left
          m_angularVelocity = -f_Xm*0.8;
          ROS_INFO("Rotate to Right");
          approachMarker();
        }
        else
          {
           m_angularVelocity = 0;
           approachMarker();
          }
        }
      else{gotTarget=false;}
     }
   ros::spinOnce();
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
  double randDist = 1.5*((double) rand() / (RAND_MAX));
  double randAngle = 90.0*((double) rand() / (RAND_MAX));
  ROS_INFO("Random Angle = %f", randAngle);
  rotateOdom(randAngle);
  ROS_INFO("Random Distance = %f", randDist);
  driveForwardOdom(randDist);
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
          reachedTarget = false;
          moveRandomly();
        }
      else
        {
          driveForwardOdom(0.2);
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



