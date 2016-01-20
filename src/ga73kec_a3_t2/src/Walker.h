#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

class	Walker	{
public:
                //	Tunable	parameters
    const static double FORWARD_SPEED_MPS	= 0.2;

    const static double MIN_SCAN_ANGLE_RAD	= -0.2 + M_PI ;//+150.0/180*M_PI;
    const static double MAX_SCAN_ANGLE_RAD	= 0.2 + M_PI; //+210.0/180*M_PI;

    const static float MIN_PROXIMITY_RANGE_M=1.0;	//	Should	be	smaller	than	sensor_msgs::LaserScan::range_max
    Walker();
    void	Moving();
private:
    ros::NodeHandle	node;
    ros::Publisher	commandPub;	//	Publisher	to	the	robot's	velocity	command	topic
    ros::Subscriber	laserSub;	//	Subscriber	to	the	robot's	laser	scan	topic
    bool	keepMoving;	//	Indicates	whether	the	robot	should	continue	moving
    void	moveForward();
    void	scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    void    rotateTurtle();
};
