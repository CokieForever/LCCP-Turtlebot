#include "stopper.hpp"
#include "geometry_msgs/Twist.h"

//Constructor
Stopper::Stopper()
{
	keepMoving = true;
    commandPub = node.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);
    laserSub = node.subscribe("/scan", 1, &Stopper::scanCallback, this);
    markerId = node.subscribe("/int8_marker_ids", 10, &Stopper::markerCallback, this);
    bumper = node.subscribe("mobile_base/events/bumper", 20, &Stopper::bumperCallback, this);
	nextId = 0;
}

//Method for a forward movement of the turtlebot
void Stopper::moveForward()
{
	geometry_msgs::Twist msg;
	msg.linear.x = FORWARD_SPEED_MPS;
	commandPub.publish(msg);
}


//Will be called if there is an obstacle or a bumper was launched
void Stopper::rotate()
{
    geometry_msgs::Twist msg;
    msg.angular.z = 0.785398;
    commandPub.publish(msg);
}

//Callback of the /scan topic. Checks wether an obstacle disturbs forward movement
void Stopper::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
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

	if(closestRange<MIN_PROXIMITY_RANGE_M)	
	{
        ROS_INFO("Obstacle!");
		keepMoving=false;
        rotate();
	}
    else
    {
        keepMoving = true;
    }
}

//Method which will be called from run_stopper.cpp
void Stopper::startMoving()
{
    ros::Rate rate(10);
    ROS_INFO("Start moving");

    while(ros::ok() && keepMoving)
    {
        moveForward();
        ros::spinOnce();
        rate.sleep();
    }
    while(!keepMoving)
    {
        rotate();
        ros::spinOnce();
    }
}

//Callback function for incoming Marker msgs. Checks the detected MarkerIDs and compare it to the next wanted ID.
void Stopper::markerCallback(const detect_marker::Markers::ConstPtr& markers_msg)
{
	int highestMarkerId = 0;
	
    for(int i=0; i<8; i++)
	{
        if((markers_msg->marker[i] >= highestMarkerId) && (markers_msg->marker[i]!=8))
        {
            ROS_INFO("Marker content: %d", markers_msg->marker[i] );
            highestMarkerId = markers_msg->marker[i];
        }
	}
	
    if(highestMarkerId == nextId && !keepMoving)
    {
        nextId =+ 1;
        ROS_INFO("next ID is now %d", nextId);
    }
}		

//Callback function for the bumper event. Stops the forward movement and iniates an rotation
void Stopper::bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& bumper_msg)
{
    ROS_INFO("You just get bumped!");
    keepMoving=false;
    rotate();
}
