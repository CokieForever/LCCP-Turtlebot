
#include "Stopper.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <stdlib.h>
#include "tf/transform_listener.h"
Stopper::Stopper()
{
  keepMoving = true;
  //Advertise a new publisher for the simulated robot's velocity command topic
  commandPub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
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
void Stopper::rotateOdom(double angle)
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


    //bool needed to check if while loop has already passed first time
    bool runFirstTime = true;

    ros::Rate rate2(10);
    //Run loop until rotation has been done
    while (!done && node.ok())
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
void Stopper::rotate()
{
    /* initialize random seed: */
    srand (time(NULL));
    geometry_msgs::Twist msg;
    ROS_INFO_STREAM("Rotation: "<<msg.angular.z);
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
  ROS_INFO("Driving Forward");
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
		rotateOdom(45);
		ros::Duration(1.5).sleep();
		moveForward();
	}
}

void Stopper::startMoving()
{
	ros::Rate rate(100);
	ROS_INFO("Start moving");
	
	//Keep spinning loop until user presses Ctrl+C or the robot got too close to an obstacle
	while(ros::ok()&&keepMoving)
	{
			moveForward();
			ros::spinOnce(); //Need to call this function often to allow ROS to process incoming messages
			rate.sleep();
	
	}
}

