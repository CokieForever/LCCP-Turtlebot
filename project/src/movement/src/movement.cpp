#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "movement.hpp"
#include "geometry_msgs/Twist.h"
#include <math.h>
/**
 * @brief Initialization of important parameters and subscription to necessary topics.
 */
Mover::Mover()
{
    m_turtleSpeed = 0.25;
    m_keepMoving = true;
    m_gotTarget = false;
    m_reachedTarget = false;
    m_nextId = 0;
    m_searchMarker = 0;
    m_finishedMarkerSearch = false;
    m_outOfSight = true;
    m_distanceToMarker = 9999;
    commandPub = node.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);
    targetReachedRequest = node.advertise<std_msgs::Empty>("targetReached", 10);
    m_driveNow = false;
    turtleStar = node.subscribe("/gotStar",1,&Mover::starCallBack,this);
    laserSub = node.subscribe("/scan", 1, &Mover::scanCallback, this);
    bumperSub = node.subscribe("mobile_base/events/bumper", 20, &Mover::bumperSubCallback, this);
    getLocationSub = node.subscribe("/markerinfo", 10, &Mover::getLocationCallback, this);
    targetFinishedSub = node.subscribe("targetFinishedTopic", 10, &Mover::targetFinishedCallback, this);
}
/**
 * @brief Callback function: when a star is recognized, the topic /gotStar triggers the execution
 * @param star std_msgs::Bool
 */
void Mover::starCallBack(const std_msgs::Bool::ConstPtr &star)
{
    if (star->data)
    {
        m_turtleSpeed *= 1.3;
    }
    else
    {
        m_turtleSpeed /= 1.3;
    }
}
/**
 * @brief Use the odometry to drive a given distance (in meters)
 * @param distance in meters
 */
void Mover::driveForwardOdom(double distance)
{
    geometry_msgs::Twist base_cmd;
    base_cmd.linear.y = base_cmd.angular.z = 0;
    base_cmd.linear.x = m_turtleSpeed;
    ros::Rate rate(100.0);
    double distanceMoved = 0;
    ros::Time startTime = ros::Time::now();
    while (((ros::Time::now()-startTime).toSec()*0.2 < distance) && node.ok() && m_keepMoving)
    {
        base_cmd.linear.y = base_cmd.angular.z = 0;
        base_cmd.linear.x = m_turtleSpeed;
        commandPub.publish(base_cmd);
        rate.sleep();
        ros::spinOnce();
        rate.sleep();
    }

}
/**
 * @brief Use the odometry to rotate the turtlebot by a given number of degrees
 * @param angle in degrees
 */
void Mover::rotateOdom(double angle)
{
    listener.waitForTransform("base_footprint", "/odom", ros::Time(0), ros::Duration(1.0));
    tf::StampedTransform start_transform;
    tf::StampedTransform current_transform;
    listener.lookupTransform("base_footprint", "/odom", ros::Time(0), start_transform);
    geometry_msgs::Twist base_cmd;
    if(angle<0)
    {
        base_cmd.linear.y = base_cmd.linear.x = 0;
        base_cmd.angular.z = 0.45;
    }
    else
    {
        base_cmd.linear.y = base_cmd.linear.x = 0;
        base_cmd.angular.z = -0.45;
    }
    bool done = false;
    tfScalar originAngle = tf::getYaw(start_transform.getRotation());
    bool runFirstTime = true;
    ros::Rate rate2(100);
    while (!done && node.ok() ) //&& !m_gotTarget
    {
        tfScalar relativeAngle, currentAngle, previousAngle;
        commandPub.publish(base_cmd);
        rate2.sleep();
        if(runFirstTime==false){previousAngle = currentAngle;}
        try
        {
            listener.lookupTransform("base_footprint", "/odom", ros::Time(0), current_transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
            break;
        }
        currentAngle = tf::getYaw(current_transform.getRotation());
        if(runFirstTime==false && fabs((currentAngle-previousAngle)) > 2.0)
        {
            currentAngle = (currentAngle + 2*M_PI);
            double difference = fabs((currentAngle-previousAngle));
        }
        relativeAngle = currentAngle - originAngle;
        relativeAngle = ((relativeAngle*180.0)/M_PI);
        runFirstTime = false;
        if(relativeAngle>angle)
        {
            done=true;
            m_keepMoving=true;

            runFirstTime =true;
        }
        ros::spinOnce();


    }
    ros::spinOnce();
}

/**
 * @brief Callback of the /scan topic. Checks wether an obstacle disturbs forward movement
 * @param scan sensor_msgs::LaserScan
 */
void Mover::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int minIndex = ceil((MIN_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);
    int maxIndex = floor((MAX_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);
    ros::Rate rate(10);
    float closestRange = scan->ranges[minIndex];
    if(scan->ranges[0]<=10 && scan->ranges[0]!=INFINITY){
        m_distanceToMarker = scan->ranges[0];
    }
    ROS_INFO("Distance to marker %f !", m_distanceToMarker);
    for(int currIndex = minIndex+1; currIndex<=maxIndex; currIndex++)
    {
        if(scan->ranges[currIndex] < closestRange && scan->ranges[currIndex] != 0.0 )
        {
            closestRange =	scan->ranges[currIndex];
        }
    }
    if(closestRange < MIN_PROXIMITY_RANGE_M)
    {
        m_keepMoving=false;
        ROS_INFO("Obstacle in %f Turn 25 degrees!", closestRange);
        rotateOdom(25);
        rate.sleep();
    }
    else
    {
        m_keepMoving = true;
    }
    ros::spinOnce();
}
/**
 * @brief Callback function for the bumperSub event. Stops the forward movement and initiates a rotation
 * @param bumperSub_msg kobuki_msgs::BumperEvent
 */
void Mover::bumperSubCallback(const kobuki_msgs::BumperEvent::ConstPtr& bumperSub_msg)
{
    if(bumperSub_msg->state)
    {
        switch(bumperSub_msg->bumper)
        {
        case 0: ROS_INFO("You just get bumped on the left!");
            break;
        case 1: ROS_INFO("You just get bumped in the middle!");
            break;
        case 2: ROS_INFO("You just get bumped on the right!");
            break;
        default:  ROS_INFO("I bumped a ghost :P");
        }
        geometry_msgs::Twist base_cmd;
        base_cmd.linear.y = base_cmd.angular.z = 0;
        base_cmd.linear.x = -1 * m_turtleSpeed;
        commandPub.publish(base_cmd);
        ros::Duration(0.5).sleep();
        m_keepMoving=false;
        rotateOdom(45);
        ros::spinOnce();
    }
}
/**
 * @brief Old method used to approach a marker given from the detect_marker node.
 * @param param_marker_id integer
 */
void Mover::approachMarker(int param_marker_id)
{
    ros::Rate rateX(20);
    geometry_msgs::Twist vel_msg;
    static float f_distance_old = m_distanceToMarker; // save
    bool b_invalid_object = false;
    vel_msg.angular.z = m_angularVelocity;
    if (m_distanceToMarker > 1.5)
        vel_msg.linear.x = 0.5;
    else
        vel_msg.linear.x = 0.3* m_distanceToMarker;
    commandPub.publish(vel_msg);
    if (f_distance_old != m_distanceToMarker)
    {
        if ((f_distance_old -  m_distanceToMarker ) > 0.1)
        {
            //this is an invalid object!!!
            b_invalid_object = true;
            //rotate the robot to the other direction to correct the mistake
            m_angularVelocity = (-1)*m_angularVelocity;
        }
        else
            b_invalid_object = false;
        f_distance_old = m_distanceToMarker;
    }
    commandPub.publish(vel_msg);
    if ((m_distanceToMarker<=0.7) && m_gotTarget && (param_marker_id == m_searchMarker) && !b_invalid_object)
    {
        if (m_searchMarker == 7)
        {
            m_finishedMarkerSearch = true;
            ROS_INFO("Reached all targets!!!");
            vel_msg.angular.z = 0;
            vel_msg.linear.x = 0;
            commandPub.publish(vel_msg);
            rotateOdom(180);
            driveForwardOdom(0.75);

        }
        ROS_INFO("Target Reached!!! The Marker is %d", m_searchMarker++);
        ROS_INFO("Searching next target: %d , distance is %f", m_searchMarker, m_distanceToMarker);
        m_gotTarget = false;
        m_reachedTarget = true;
        rotateOdom(180);
        ROS_INFO("Turned 180 degrees");
    }

    rateX.sleep();
    ros::spinOnce();
}
/**
 * @brief Callback function that is triggered when a marker array is sent on the /markerinfo topic. Contains commented code that uses the old approach marker function, in case tf doesn't work well.
 * @param marker_msg detect_marker::MarkersInfos
 */
void Mover::getLocationCallback(const detect_marker::MarkersInfos::ConstPtr &marker_msg)
{
    if (marker_msg->infos.size() || m_gotTarget && m_keepMoving)
    {
        int i_array_length = marker_msg->infos.size();
        float f_Xm = 0.0;
        int i_marker_id = 0;
        m_outOfSight = false;
        for (int i=0; i<i_array_length; i++)
        {
            if (marker_msg->infos[i].id == m_searchMarker)
            {
                f_Xm = marker_msg->infos[i].x;
                m_driveNow = false;
                i_marker_id = marker_msg->infos[i].id;
                m_gotTarget = true;
                m_reachedTarget = false;
            }
        }
        ros::Rate rate(50);
        while (m_gotTarget)
        {
            tf::StampedTransform transform_marker;
            char goalMarker[100];
            snprintf(goalMarker,100, "deadreckoning_markerpos_%d",m_searchMarker);
            try{
                m_coordinateListener.lookupTransform("deadreckoning_robotpos", goalMarker,
                                                     ros::Time(0), transform_marker);
            }
            catch (tf::TransformException &ex) {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }
            geometry_msgs::Twist vel_msg;
            float f_distance_to_marker = sqrt(pow(transform_marker.getOrigin().x(), 2) +
                                              pow(transform_marker.getOrigin().y(), 2));
            if ( (f_distance_to_marker <= 0.5))
            {
                m_gotTarget = false;
                vel_msg.angular.z = 0;
                vel_msg.linear.x = 0;
                m_searchMarker++;
                ROS_INFO("I'm too close to the marker, search next: %d", m_searchMarker);
            }
            else
            {
                if (m_keepMoving && !m_driveNow)
                {
                    vel_msg.angular.z = 1.0 * atan2(transform_marker.getOrigin().y(),
                                                    transform_marker.getOrigin().x());
                    vel_msg.linear.x = 0.2 * sqrt(pow(transform_marker.getOrigin().x(), 2) +
                                                  pow(transform_marker.getOrigin().y(), 2));
                }

            }
            commandPub.publish(vel_msg);
            ros::spinOnce();
            rate.sleep();
        }
        /* Back-up plan: if it doesn't work with tf use old function approachMarker();
        if (!m_reachedTarget && m_gotTarget && !m_outOfSight)
        {
            //adjust robot, so the marker actually is in the center
            if (f_Xm < 0)
            {
                //rotate bot to the right
                m_angularVelocity = -f_Xm*0.8;
                approachMarker(i_marker_id);
            }
            else if (f_Xm > 0)
            {
                //rotate bot to the left
                m_angularVelocity = -f_Xm*0.8;
                approachMarker(i_marker_id);
            }
            else
            {
                m_angularVelocity = 0;
                approachMarker(i_marker_id);
            }
        }
        else
        {
            m_gotTarget=false;
        }*/
    }
    ros::spinOnce();
}
/**
 * @brief Callback function that is called when the turtlebot reached all targets.
 * @param empty std_msgs::Empty
 */
void Mover::targetFinishedCallback(const std_msgs::EmptyConstPtr empty)
{
    m_gotTarget = false;
    rotateOdom(180);
    driveForwardOdom(0.5);
    rotateOdom(358);
}
/**
 * @brief The parameters of the random movement are given randomly.
 */
void Mover::moveRandomly()
{
    srand(time(NULL));
    double randDist = 1.0*((double) rand() / (RAND_MAX));
    double randAngle = 90.0*((double) rand() / (RAND_MAX));
    rotateOdom(randAngle);
    driveForwardOdom(randDist);
    ROS_INFO("RAndom Walk");
}
/**
 * @brief Method which will be called from run_stopper.cpp to start the movement.
 */
void Mover::startMoving()
{
    ros::Rate rate(100);
    ROS_INFO("Start moving");
    while(ros::ok() && m_keepMoving)
    {
        if(m_gotTarget==false)
        {
            m_reachedTarget = false;
            moveRandomly();
        }
        else
        {
            if (m_finishedMarkerSearch)
            {
                ROS_INFO("Done!");
                break;
            }
        }
        ros::spinOnce();
        rate.sleep();
    }
    while(!m_keepMoving)
    {
        rotateOdom(30);
        ros::spinOnce();
    }
}


