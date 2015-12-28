//ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
//OpenCV
#include "opencv2/objdetect/objdetect.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "aruco/markerdetector.h"
#include "aruco/cvdrawingutils.h"
#include "aruco/cameraparameters.h"
#include <stdio.h>
#include <iostream>


cv_bridge::CvImageConstPtr cv_ptr;
aruco::MarkerDetector detector;
aruco::CvDrawingUtils drawUtil;
aruco::CameraParameters camParams;
void rgbCallback(const sensor_msgs::ImageConstPtr& msg)
{

	try
	{
		cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);//toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

	}
	catch (cv_bridge::Exception& ex)
	{
		ROS_ERROR("cv_bridge exception: %s", ex.what());
		exit(-1);
	}
    cv::Mat frame = cv_ptr->image;
    std::vector<aruco::Marker> marker;

    detector.detect(frame, marker, camParams);
    int laenge = marker.size();
    ROS_INFO("%d", laenge);
    cv::Scalar color(0, 30, 30, 30);

    if(laenge!=0)
    {
        for(int i=0; i<laenge; i++)
        {
            marker[i].draw(frame, color);
        }
    }
    /*Mat frame;

       if(cv_ptr)
       {
          frame = cv_ptr->image;
          ROS_INFO("du bist in der while schleife");
       //-- 3. Apply the classifier to the frame
           if( !frame.empty())
           {detectAndDisplay(frame);}
       }*/
    cv::imshow("RGB", frame);
	cv::waitKey(30); //!!!!!!
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "kinectViewer");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/camera/rgb/image_rect_color", 1, rgbCallback);
	ros::spin();
	return 0;
}
