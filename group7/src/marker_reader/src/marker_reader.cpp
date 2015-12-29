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
#include "aruco/cameraparameters.h"
#include <stdio.h>
#include <iostream>

/**Global variables**/

cv_bridge::CvImageConstPtr cv_ptr;

cv::Mat frame;
cv::Scalar color(255, 155, 0, 0);

std::vector<aruco::Marker> marker;
aruco::MarkerDetector detector;
aruco::CameraParameters camParams;
int nextId=0;


/************  Callbackfunction of /camera/rgb/image_rect_color subscribing  *******************/
/***********************************************************************************************/
/*  rgbCallback() checks for aruco markers in input-image and draws them into the input-image  */

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

    //Fill image into frame
    frame = cv_ptr->image;

    //Check frame for aruco markers
    detector.detect(frame, marker);

    //get number of markers detected
    int laenge = marker.size();
    ROS_INFO("%d", laenge);

    //when the marker-vector isn't empty, the founded markers will be drawn into the image
    if(laenge!=0)
    {
        for(int i=0; i<laenge; i++)
        {
            marker[i].draw(frame, color);
            ROS_INFO("%d", marker[i].id);
        }
    }


    cv::imshow("RGB", frame);
	cv::waitKey(30); //!!!!!!
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "MarkerReader");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/camera/rgb/image_rect_color", 1, rgbCallback);
	ros::spin();
	return 0;
}
