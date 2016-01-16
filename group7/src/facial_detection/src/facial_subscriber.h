#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

class face_detector{
	public:

	face_detector();
	
	private:

	ros::NodeHandle node;
	ros::Subscriber	CameraSub;

