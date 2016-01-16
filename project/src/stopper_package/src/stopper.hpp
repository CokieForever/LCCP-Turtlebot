#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "detect_marker/Markers.h"
#include "kobuki_msgs/BumperEvent.h"
class Stopper
{
	public://Tunable parameters

        const static double FORWARD_SPEED_MPS = 0.2;
        const static double MIN_SCAN_ANGLE_RAD = -0.2 +M_PI;
        const static double MAX_SCAN_ANGLE_RAD = 0.2 + M_PI;
        const static float  MIN_PROXIMITY_RANGE_M = 1.5;	//Should be smaller than sensor_msgs::LaserScan::range_max
	    Stopper();
		void startMoving();

	private:  	
  	
		ros::NodeHandle	node;
		ros::Publisher commandPub;	//	Publisher to the robot's velocity command topic	
		ros::Subscriber	laserSub;	// Subscriber to the robot's laser scan topic
        ros::Subscriber markerId;   // Subscriber to the detect_markers topic including the founded aruco-markers
        ros::Subscriber bumper;     // Subscriber to the robot's bumber event topic

		bool keepMoving; //Indicates wether the robot should continue moving
        int  nextId;
		void moveForward();
		void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
        void rotate();
        void markerCallback(const detect_marker::Markers::ConstPtr& marker_msg);
        void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& bumper_msg);
};
