#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "movement.hpp"
#include "geometry_msgs/Twist.h"
#include <math.h>

Mover::Mover()
{
    //Variables
    m_keepMoving = true;
    m_gotTarget = false;
    m_reachedTarget = false;
    m_nextId = 0;
    m_searchMarker = 0;
    m_finishedMarkerSearch = false;
    m_outOfSight = true;
    //distance between marker and turtlebot
    m_distanceToMarker = 9999;
    //Publishers
    commandPub = node.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);
    targetReachedRequest = node.advertise<std_msgs::Empty>("targetReached", 10);

    //Subscribers
    laserSub = node.subscribe("/scan", 1, &Mover::scanCallback, this);
    bumperSub = node.subscribe("mobile_base/events/bumper", 20, &Mover::bumperSubCallback, this);
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
    base_cmd.linear.x = 0.25;

    ros::Rate rate(10.0);
    double distanceMoved = 0;
    ros::Time startTime = ros::Time::now();

    //while((t[s]*v[m/s] = travelled distance[m] < distance[m] )
    while (((ros::Time::now()-startTime).toSec()*0.2 < distance) && node.ok() && m_keepMoving)
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
    // ROS_INFO("original angle %f", originAngle);

    //bool needed to check if while loop has already passed first time
    bool runFirstTime = true;

    ros::Rate rate2(10);
    //Run loop until rotation has been done
    while (!done && node.ok() && !m_gotTarget)
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
            //   ROS_INFO("difference %f", difference);
        }

        //calculate already absolved angle
        relativeAngle = currentAngle - originAngle;
        relativeAngle = ((relativeAngle*180.0)/M_PI);
        runFirstTime = false;

        //check if rotation is done
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


//Callback of the /scan topic. Checks wether an obstacle disturbs forward movement
void Mover::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int minIndex = ceil((MIN_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);
    int maxIndex = floor((MAX_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);
    //float closestRange = scan->ranges[minIndex];
    float closestRange;
    //save distance between robot and marker (which is in the center)
    if(scan->ranges[0]<=10 && scan->ranges[0]!=INFINITY){
        m_distanceToMarker = scan->ranges[0];
    }
    ROS_INFO("Distance to marker %f !", m_distanceToMarker);
    for(int currIndex = minIndex+1; currIndex<=maxIndex; currIndex++)
    {
        if(scan->ranges[currIndex] < closestRange)
        {
            closestRange =	scan->ranges[currIndex];
        }
    }
    //ROS_INFO("closest range = %f", closestRange);
    if(closestRange < MIN_PROXIMITY_RANGE_M)
    {
        m_keepMoving=false;
        //rotate only if not approaching target
        ROS_INFO("Obstacle in %f Turn 25 degrees!", closestRange);
        rotateOdom(25.0);
    }
    else
    {
        m_keepMoving = true;
    }
    ros::spinOnce();
}

//Callback function for the bumperSub event. Stops the forward movement and initiates an rotation
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

        //Move backwards
        geometry_msgs::Twist base_cmd;
        base_cmd.linear.y = base_cmd.angular.z = 0;
        base_cmd.linear.x = -0.2;
        commandPub.publish(base_cmd);//drive backwards
        ros::Duration(0.5).sleep(); // sleep for half a second
        m_keepMoving=false;
        rotateOdom(45);
        ros::spinOnce();
    }
}

void Mover::approachMarker(int param_marker_id)
{
    ros::Rate rateX(20);
    geometry_msgs::Twist vel_msg;
    //save old distance to check if object is valid
    static float f_distance_old = m_distanceToMarker; // save
    bool b_invalid_object = false;

    vel_msg.angular.z = m_angularVelocity;
    if (m_distanceToMarker > 1.5)
        vel_msg.linear.x = 0.5;
    else
        vel_msg.linear.x = 0.3* m_distanceToMarker;
    //decrease velocity as bot gets closer to target
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
    //return the robot to its correct course
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

void Mover::getLocationCallback(const detect_marker::MarkersInfos::ConstPtr &marker_msg)
{
    if (marker_msg->infos.size() || m_gotTarget)
    {
        //ROS_INFO("location callback!");
        //map the coordiante to the center
        int i_array_length = marker_msg->infos.size();
        float f_Xm = 0.0;
        //float f_Ym = marker_msg->infos[i].y;
        int i_marker_id = 0;
        m_outOfSight = false;
        for (int i=0; i<i_array_length; i++)
        {
            //check if marker id is equal to the next id that the bot is searching
            if (marker_msg->infos[i].id == m_searchMarker)
            {
                f_Xm = marker_msg->infos[i].x;
                //got a new target!
                i_marker_id = marker_msg->infos[i].id;
                m_gotTarget = true;
                m_reachedTarget = false;
                m_outOfSight = true;
            }
        }

        //ROS_INFO("Location of marker: %f", f_Xm);
        //was if (!m_reachedTarget && f_Xm!=0.0 && m_gotTarget)
        while (m_gotTarget) //  was && m_outOfSight
        {
            ROS_INFO("TF: SWITCHING TO TF");
            //move to transform approach
            ros::Rate rate(10);
            //TF - decrease distance to target
            tf::StampedTransform transform_marker;
            char goalMarker[100];
            // listen to tf of next marker position
            snprintf(goalMarker,100, "deadreckoning_markerpos_%d",m_searchMarker);
            try{
                //Listen to transformations: Position of robot and position of marker
                m_coordinateListener.lookupTransform("deadreckoning_robotpos", goalMarker,
                                                     ros::Time(0), transform_marker);
            }
            catch (tf::TransformException &ex) {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }
            geometry_msgs::Twist vel_msg;
            // decrease distance between robot and marker, while avoiding obstacles;
            float f_distance_to_marker = sqrt(pow(transform_marker.getOrigin().x(), 2) +
                                              pow(transform_marker.getOrigin().y(), 2));
            if ( (f_distance_to_marker <= 0.5)) //m_distanceToMarker <= 0.7 &&
            {
                m_gotTarget = false;
                vel_msg.angular.z = 0;
                vel_msg.linear.x = 0;
                //search for next marker
                m_searchMarker++;
                ROS_INFO("I'm too close to the marker, search next: %d", m_searchMarker);
            }
            else
            {
                vel_msg.angular.z = 2.0 * atan2(transform_marker.getOrigin().y(),
                                                transform_marker.getOrigin().x());
                vel_msg.linear.x = 0.2 * sqrt(pow(transform_marker.getOrigin().x(), 2) +
                                              pow(transform_marker.getOrigin().y(), 2));
            }
            commandPub.publish(vel_msg);
            ros::spinOnce();
            rate.sleep();
        }
        /* Back-up plan
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

void Mover::targetFinishedCallback(const std_msgs::EmptyConstPtr empty)
{
    m_gotTarget = false;
    rotateOdom(180);
    driveForwardOdom(0.5);
    rotateOdom(358);
}
void Mover::moveRandomly()
{

    srand(time(NULL));
    double randDist = 1.0*((double) rand() / (RAND_MAX));
    double randAngle = 90.0*((double) rand() / (RAND_MAX));
    // ROS_INFO("Random Angle = %f", randAngle);
    rotateOdom(randAngle);
    //ROS_INFO("Random Distance = %f", randDist);
    driveForwardOdom(randDist);
    ROS_INFO("RAndom Walk");
}
//Method which will be called from run_stopper.cpp
void Mover::startMoving()
{
    ros::Rate rate(10);
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
            //   driveForwardOdom(0.2);
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

