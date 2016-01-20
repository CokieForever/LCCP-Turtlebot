#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
//#include "detect_marker/Markers.h"
#include "kobuki_msgs/BumperEvent.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/Image.h"
class Mover
{
public://Tunable parameters

  const static double FORWARD_SPEED_MPS = 0.2;
  const static double MIN_SCAN_ANGLE_RAD = -0.5;
  const static double MAX_SCAN_ANGLE_RAD = 0.5;
  const static float  MIN_PROXIMITY_RANGE_M = 0.75;	//Should be smaller than sensor_msgs::LaserScan::range_max
  Mover();
  void startMoving();
  tf::TransformListener listener;

private:

	//Declare NodeHandler
	ros::NodeHandle	node;

	/*Declare Publisher
	 * commandPub
	 * targetReachedRequest
	 * */

	ros::Publisher commandPub;	//	Publisher to the robot's velocity command topic
	ros::Publisher targetReachedRequest; // Publishes a "true" state

	/*Declare Subscriber
	 * laserSub
	 * bumperSub
	 * imageSub
	 * getLocationSub
	 * targetFinished
	 * */
	ros::Subscriber	laserSub;	// Subscriber to the robot's laser scan topic
	//ros::Subscriber markerId;   // Subscriber to the detect_markers topic including the founded aruco-markers
	ros::Subscriber bumperSub;     // Subscriber to the robot's bumber event topic
	//ros::Subscriber imageSub;
	ros::Subscriber getLocationSub;
	ros::Subscriber targetFinishedSub;

	bool keepMoving; //Indicates wether the robot should continue moving
	bool gotTarget;
	geometry_msgs::Vector3Stamped target;
	int  nextId;
	void moveRandomly();

	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
	void bumperSubCallback(const kobuki_msgs::BumperEvent::ConstPtr& bumper_msg);
	//void rgbCallback(const sensor_msgs::ImageConstPtr &msg);
	void getLocationCallback(const geometry_msgs::Vector3StampedConstPtr &vector);
	void targetFinishedCallback(const std_msgs::EmptyConstPtr empty);
	void driveForwardOdom(double distance);
	void rotateOdom(double angle);
};
