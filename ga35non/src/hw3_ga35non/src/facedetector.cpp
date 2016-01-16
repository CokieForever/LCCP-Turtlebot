#include <ros/ros.h>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "utilities.h"

using namespace std;
using namespace cv;

class MyLittleFaceDetector
{
	private:
		static const string WINDOW_NAME;
		ros::NodeHandle& m_nodeHandle;
		ros::Subscriber m_kinectSub;
		CascadeClassifier m_faceClassifier;
		CascadeClassifier m_eyesClassifier;
		bool m_ok;

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
			
			detectFaceAndDisplay(imgPtr->image);
		}

		//From http://docs.opencv.org/2.4/doc/tutorials/objdetect/cascade_classifier/cascade_classifier.html
		void detectFaceAndDisplay(Mat img)
		{
			std::vector<Rect> faceRects;
			Mat grayImg;

			cvtColor(img, grayImg, CV_BGR2GRAY);
			equalizeHist(grayImg, grayImg);

			//-- Detect faces
			m_faceClassifier.detectMultiScale(grayImg, faceRects, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30));

			for (size_t i=0 ; i < faceRects.size() ; i++)
			{
				Point center(faceRects[i].x + faceRects[i].width*0.5, faceRects[i].y + faceRects[i].height*0.5);
				ellipse(img, center, Size(faceRects[i].width*0.5, faceRects[i].height*0.5), 0, 0, 360, Scalar(255, 0, 255), 4, 8, 0);

				Mat faceROI = grayImg(faceRects[i]);
				std::vector<Rect> eyesRects;

				//-- In each face, detect eyes
				m_eyesClassifier.detectMultiScale(faceROI, eyesRects, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30));

				for (size_t j=0 ; j < eyesRects.size() ; j++ )
				{
					Point center(faceRects[i].x + eyesRects[j].x + eyesRects[j].width*0.5, faceRects[i].y + eyesRects[j].y + eyesRects[j].height*0.5 );
					int radius = cvRound((eyesRects[j].width + eyesRects[j].height) * 0.25);
					circle(img, center, radius, Scalar(255, 0, 0), 4, 8, 0);
				}
			}

			//-- Show what you got
			imshow(WINDOW_NAME, img);
			waitKey(30);
		}

	public:
		MyLittleFaceDetector(ros::NodeHandle& nodeHandle): m_nodeHandle(nodeHandle), m_ok(false)
		{
			ROS_INFO("Creating window...");
			namedWindow(WINDOW_NAME, WINDOW_AUTOSIZE);
			waitKey(30);
			
			ROS_INFO("Loading classifiers...");
			if (!m_faceClassifier.load("/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml"))
			{
				ROS_ERROR("Unable to load the face cascade classifier.");
				return;
			}
   			if (!m_eyesClassifier.load("/usr/share/opencv/haarcascades/haarcascade_eye_tree_eyeglasses.xml"))
			{
				ROS_ERROR("Unable to load the face cascade classifier.");
				return;
			}

			ROS_INFO("Creating subscribers for the kinect sensor...");
			m_kinectSub = m_nodeHandle.subscribe("/depth/image_raw", 10, &MyLittleFaceDetector::imageCallback, this);
			ros::Rate rate(10);
			while (ros::ok() && m_kinectSub.getNumPublishers() <= 0)
				rate.sleep();
			checkRosOk_v();

			ROS_INFO("Ok, everything's ready.");
			m_ok = true;
		}

		bool isOK()
		{
			return m_ok;
		}

		void detect()
		{
			ROS_INFO("Started detection.");
			ros::spin();
		}
};

const string MyLittleFaceDetector::WINDOW_NAME = "Face Detector";

int main(int argc, char **argv)
{
	ros::init(argc, argv, "facedetector");
	ROS_INFO("Initialized ROS.");

	ros::NodeHandle nodeHandle;
	MyLittleFaceDetector faceDetector(nodeHandle);
	if (!faceDetector.isOK())
		return -1;
	faceDetector.detect();

	ROS_INFO("Bye!");
	return 0;
}
