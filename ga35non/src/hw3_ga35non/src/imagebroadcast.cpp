#include <ros/ros.h>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "utilities.h"

using namespace std;
using namespace cv;

class MyLittleImageBroadcaster
{
	private:
		ros::NodeHandle& m_nodeHandle;
		ros::Publisher m_kinectPub;
		CvCapture* m_capture;
		bool m_ok;
		
	public:
		MyLittleImageBroadcaster(ros::NodeHandle& nodeHandle, string videoFilePath = ""): m_nodeHandle(nodeHandle), m_ok(false)
		{
			if (videoFilePath == "")
			{
				ROS_INFO("Initializing video capture...");
				m_capture = cvCaptureFromCAM(0);
			}
			else
			{
				ROS_INFO("Loading video file \"%s\"...", videoFilePath.c_str());
				m_capture = cvCaptureFromFile(videoFilePath.c_str());
			}

			if (m_capture == NULL)
			{
				ROS_ERROR("Unable to access video data.");
				return;
			}

			ROS_INFO("Creating publisher...");
			m_kinectPub = m_nodeHandle.advertise<sensor_msgs::Image>("/depth/image_raw", 10);
			ros::Rate rate(10);
			while (ros::ok() && m_kinectPub.getNumSubscribers() <= 0)
				rate.sleep();
			checkRosOk_v();

			ROS_INFO("Ok, everything's ready.");
			m_ok = true;
		}

		bool isOK()
		{
			return m_ok;
		}
		
		bool broadcast()
		{
			ROS_INFO("Started broadcast.");
			Mat frame;

			ros::Rate rate(25);
			while(ros::ok())
			{
				frame = cvQueryFrame(m_capture);

				if (!frame.empty())
				{
					cv_bridge::CvImage cvImage;
					//cvImage.header   = in_msg->header;
					cvImage.encoding = sensor_msgs::image_encodings::BGR8;
					cvImage.image    = frame;
					m_kinectPub.publish(cvImage.toImageMsg());
				}
				else
				{
					ROS_ERROR("No captured frame. Stopping.");
					return false;
				}

				rate.sleep();
			}
		
			return true;
		}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "imagebroadcast");
	ROS_INFO("Initialized ROS.");

	ros::NodeHandle nodeHandle;
	MyLittleImageBroadcaster imageBroadcaster(nodeHandle, argc > 1 ? argv[1] : "");
	if (!imageBroadcaster.isOK())
		return -1;
	imageBroadcaster.broadcast();

	ROS_INFO("Bye!");
	return 0;
}
