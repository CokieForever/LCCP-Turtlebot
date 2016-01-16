//ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
//OpenCV
#include <opencv2/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

void rgbCallback(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImageConstPtr cv_ptr;
	ROS_INFO("du bist in dem RGB Callback");
	try
	{
		cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);//toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		ROS_INFO("try hat stattgefunden");

	}
	catch (cv_bridge::Exception& ex)
	{
		ROS_ERROR("cv_bridge exception: %s", ex.what());
		exit(-1);
	}
	cv::imshow("RGB", cv_ptr->image);
	cv::waitKey(30); //!!!!!!
}
/*
void depthCallback(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImageConstPtr cv_ptr;
	ROS_INFO("du bist in dem depth Callback");

	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
	}
	catch (cv_bridge::Exception& ex)
	{
		ROS_ERROR("cv_bridge exception: %s", ex.what());
		exit(-1);
	}
	cv::imshow("DEPTH", cv_ptr->image);
	cv::waitKey(30);
}
*/
int main(int argc, char **argv)
{
	ros::init(argc, argv, "kinectViewer");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/camera/rgb/image_rect_color", 1, rgbCallback);
	/*ros::Subscriber depth =	n.subscribe("/camera/depth/image", 1, depthCallback);*/
	ros::spin();
	return 0;
}


