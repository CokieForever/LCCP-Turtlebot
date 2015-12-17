#include "stopper.h"
#include "geometry_msgs/Twist.h"

const double Stopper::FORWARD_SPEED_MPS = 0.5;
const double Stopper::MIN_SCAN_ANGLE_RAD = -30.0/180*M_PI;
const double Stopper::MAX_SCAN_ANGLE_RAD = +30.0/180*M_PI;
const float Stopper::MIN_PROXIMITY_RANGE_M = 1.0; // Should be smaller than sensor_msgs::LaserScan::range_max

Stopper::Stopper(): m_keepMoving(true)
{
	// Advertise a new publisher for the simulated robot's velocity command topic
	commandPub = node.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);

	// Subscribe to the simulated robot's laser scan topic
	laserSub = node.subscribe("/scan", 1, &Stopper::scanCallback, this);
}

// Send a velocity command
void Stopper::moveForward()
{
	geometry_msgs::Twist msg; // The default constructor will set all commands to 0
	msg.linear.x = FORWARD_SPEED_MPS;
	commandPub.publish(msg);
}

// Process the incoming laser scan message
void Stopper::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	// Find the closest range between the defined minimum and maximum angles
	double startAngle = std::min((double)scan->angle_max-scan->angle_increment, std::max((double)scan->angle_min, MIN_SCAN_ANGLE_RAD));
	double endAngle = std::min((double)scan->angle_max-scan->angle_increment, std::max((double)scan->angle_min, MAX_SCAN_ANGLE_RAD));
	int minIndex = ceil((startAngle - scan->angle_min) / scan->angle_increment);
	int maxIndex = floor((endAngle - scan->angle_min) / scan->angle_increment);

	float closestRange = scan->ranges[minIndex];
	for (int i=minIndex+1 ; i <= maxIndex ; i++)
	{
		if (scan->ranges[i] < closestRange)
			closestRange = scan->ranges[i];
	}

	ROS_INFO_STREAM("Closest range: " << closestRange);

	if (closestRange < MIN_PROXIMITY_RANGE_M)
	{
		ROS_INFO("Stop!");
		m_keepMoving = false;
	}
}

void Stopper::startMoving()
{
	ros::Rate rate(10);
	ROS_INFO("Start moving");
	
	// Keep spinning loop until user presses Ctrl+C or the robot got too close to an obstacle
	while (ros::ok() && m_keepMoving)
	{
		moveForward();
		ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
		rate.sleep();
	}
}

int main(int argc, char **argv)
{
	// Initiate new ROS node named "stopper"
	ros::init(argc, argv, "stopper");
	ROS_INFO("Initialized ROS.");
	
	// Create new stopper object
	Stopper stopper;
	// Start the movement
	stopper.startMoving();
	
	return 0;
};

