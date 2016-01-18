#include <ros/ros.h>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "utilities.h"

using namespace std;
using namespace cv;

class MyLittleCamRecorder
{
	private:
		static const string WINDOW_NAME;
		ros::NodeHandle& m_nodeHandle;
		ros::Subscriber m_camSub;
		std::string m_dirPath;
		bool m_ok;
		unsigned int m_frameID;

		//Inspired from http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
		void imageCallback(const sensor_msgs::ImageConstPtr& msg)
		{
			cv_bridge::CvImagePtr imgPtr;
			try
			{
				imgPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
			}
			catch (cv_bridge::Exception& e)
			{
				ROS_ERROR("cv_bridge exception: %s", e.what());
				return;
			}
			
			saveAndDisplay(imgPtr->image);
		}

		//From http://docs.opencv.org/2.4/doc/tutorials/objdetect/cascade_classifier/cascade_classifier.html
		void saveAndDisplay(Mat img)
		{
			char imgName[10];
			snprintf(imgName, sizeof(imgName), "%05d.png", m_frameID);

			imwrite(m_dirPath + std::string(imgName), img);
			m_frameID++;

			imshow(WINDOW_NAME, img);
			waitKey(30);
		}

	public:
		MyLittleCamRecorder(ros::NodeHandle& nodeHandle, std::string dirPath): m_nodeHandle(nodeHandle), m_dirPath(dirPath), m_frameID(0), m_ok(false)
		{
			ROS_INFO("Creating window...");
			namedWindow(WINDOW_NAME, WINDOW_AUTOSIZE);
			waitKey(30);
			
			ROS_INFO("Creating subscribers for the camera data...");
			m_camSub = m_nodeHandle.subscribe("/camera/rgb/image_raw", 10, &MyLittleCamRecorder::imageCallback, this);
			ros::Rate rate(10);
			while (ros::ok() && m_camSub.getNumPublishers() <= 0)
				rate.sleep();
			checkRosOk_v();

			ROS_INFO("Ok, everything's ready.");
			m_ok = true;
		}

		bool isOK()
		{
			return m_ok;
		}

		void record()
		{
			ROS_INFO("Started recording.");
			ros::spin();
		}
};

const string MyLittleCamRecorder::WINDOW_NAME = "Cam recorder";

int main(int argc, char **argv)
{
	ros::init(argc, argv, "camrecorder");
	ROS_INFO("Initialized ROS.");

	ros::NodeHandle nodeHandle;
	MyLittleCamRecorder recorder(nodeHandle, "/home/ros/record/");
	if (!recorder.isOK())
		return -1;
	recorder.record();

	ROS_INFO("Bye!");
	return 0;
}
