#include	"Walker.h"
#include	"geometry_msgs/Twist.h"

Walker::Walker()
{
        keepMoving	=	true;
        //	Advertise	a	new	publisher	for	the	simulated	robot's	velocity	command	topic
        commandPub	=	node.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",	10);
        //	Subscribe	to	the	simulated	robot's	laser	scan	topic
        laserSub	=	node.subscribe("scan",	1,	&Walker::scanCallback,	this);
}

//	Send	a	velocity	command
void	Walker::moveForward()	{
        geometry_msgs::Twist	msg;	//	The	default	constructor	will	set	all	commands	to	0
        msg.linear.x	=	FORWARD_SPEED_MPS;
        commandPub.publish(msg);
}
void    Walker::rotateTurtle()
{
   geometry_msgs::Twist msg;
   msg.linear.x = 0;
   msg.linear.y = 0;
   msg.linear.z = 0;
   msg.angular.x = 0;
   msg.angular.y = 0;
   msg.angular.z = 0.5;
   commandPub.publish(msg);
}

void	Walker::scanCallback(const	sensor_msgs::LaserScan::ConstPtr&	scan)
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
                keepMoving	=	false; //set to false
        }
        else
            keepMoving = true;


}

void	Walker::Moving()
{
        ros::Rate	rate(100);

        //	Keep	spinning	loop	until	user	presses	Ctrl+C	or	the	robot	got	too	close	to	an	obstacle
        while	(ros::ok()	&&	keepMoving)	{
                moveForward();
                ros::spinOnce();	//	Need	to	call	this	function	often	to	allow	ROS	to	process	incoming messages
                rate.sleep();
        }
        // cannot keep on moving - start rotation

        while(!keepMoving)
        {
            //rotate
            Walker::rotateTurtle();
            ros::spinOnce();
        }
}






