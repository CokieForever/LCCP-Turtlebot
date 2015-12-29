#include "stopper.hpp"
#include "geometry_msgs/Twist.h"

Stopper::Stopper()
{
	keepMoving = true;
    commandPub = node.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);
    laserSub = node.subscribe("/scan", 1, &Stopper::scanCallback, this);
    //markerId = node.subscribe("int8_marker_ids", 10, &Stopper::markerCallback, this);
	nextId = 0;
}

void Stopper::moveForward()
{
	geometry_msgs::Twist msg;
	msg.linear.x = FORWARD_SPEED_MPS;
	commandPub.publish(msg);
}

void Stopper::rotate()
{
    geometry_msgs::Twist msg;
    msg.angular.z = 0.785398;
    commandPub.publish(msg);
}

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
    ROS_INFO("%f", closestRange);
	if(closestRange<MIN_PROXIMITY_RANGE_M)	
	{
		ROS_INFO("Stop!");
		keepMoving=false;
        rotate();
	}
    else
    {
        keepMoving = true;
    }
}

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
/*void markerCallback(const detect_marker::Markers::ConstPtr& markers_msg)
{
	int highestMarkerId = 0;
	
	for(int i=0; i<8, i++)
	{
		if(highestMarkerId < marker_msg->marker[i] || marker_msg->marker[i]!=8)
		{highestMarkerId = marker_msg->marker[i];}
	}
	
	if(highestMarker == nextId && !keepMoving)
	{nextId =+ 1; ROS_INFO("next ID is now %d", nextId);}
}		
*/
