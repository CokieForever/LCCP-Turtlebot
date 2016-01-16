#include	"Stopper.h"
#include	"geometry_msgs/Twist.h"

Stopper::Stopper()
{	
	completeRotation=false;
        keepMoving	=	true;
        //	Advertise	a	new	

	//for	the	simulated	robot's	velocity	command	topic
        commandPub	=	node.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity",	10);
        //	Subscribe	to	the	simulated	robot's	laser	scan	topic
        laserSub	=	node.subscribe("scan",	1,	&Stopper::scanCallback,	this);
}
//	Send	a	velocity	command
void	Stopper::moveForward()	{
        geometry_msgs::Twist	msg;	//	The	default	constructor	will	set	all	commands	to	0
        msg.linear.x	=	FORWARD_SPEED_MPS;
        commandPub.publish(msg);
}


void Stopper::rotation()
{
  geometry_msgs::Twist vel;

  vel.linear.x=0;
  vel.linear.y=0;
  vel.linear.z=0;

  vel.angular.x=0;
  vel.angular.y=0;

  vel.angular.z = ROTATION_SPEED;
  commandPub.publish(vel);

  
}

void	Stopper::scanCallback(const	sensor_msgs::LaserScan::ConstPtr&	scan)
{
        //	Find	the	closest	range	between	the	defined	minimum	and	maximum	angles
        int	minIndex	=	ceil((MIN_SCAN_ANGLE_RAD	-	scan->angle_min)	/	scan->angle_increment);
        int	maxIndex	=	floor((MAX_SCAN_ANGLE_RAD	-	scan->angle_min)	/	scan->angle_increment);
        float	closestRange	=	scan->ranges[minIndex];
        for	(int	currIndex	=	minIndex	+	1;	currIndex	<=	maxIndex;	currIndex++)	{
                if	(scan->ranges[currIndex]	<	closestRange)	{
                        closestRange	=	scan->ranges[currIndex];
                }
        }
        ROS_INFO_STREAM("Closest	range:	"	<<	closestRange);
        if	(closestRange	<	MIN_PROXIMITY_RANGE_M)	{
                ROS_INFO("Stop!");
                keepMoving	=	false;
		ROS_INFO("ROTATION!");
		//
               
        }
else 
{
keepMoving=true;}

}

void	Stopper::Moving()
{
        ros::Rate	rate(100);
        ROS_INFO("Start	moving");
        //	Keep	spinning	loop	until	user	presses	Ctrl+C	or	the	robot	got	too	close	to	an	obstacle
        while	(ros::ok()	&&	keepMoving)	{
                moveForward();
                ros::spinOnce();	//	Need	to	call	this	function	often	to	allow	ROS	to	process	incoming messages
                rate.sleep();
        }
	while (!keepMoving && ros::ok())
	{
		rotation();
		ros::spinOnce();
		rate.sleep();
}

}






