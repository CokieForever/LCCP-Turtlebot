#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

class Stopper
{
	public:
		// Tunable parameters
		const static double FORWARD_SPEED_MPS;
		const static double MIN_SCAN_ANGLE_RAD;
		const static double MAX_SCAN_ANGLE_RAD;
		const static float MIN_PROXIMITY_RANGE_M;
		Stopper();
		void startMoving();

	private:
		ros::NodeHandle node;
		ros::Publisher commandPub;	// Publisher to the robot's velocity command topic
		ros::Subscriber laserSub;	// Subscriber to the robot's laser scan topic
		bool m_keepMoving;			// Indicates whether the robot should continue moving
		void moveForward();
		void scanCallback(const	sensor_msgs::LaserScan::ConstPtr& scan);
};
