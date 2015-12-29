#ifndef DETECTMARKER_H
#define DETECTMARKER_H
//aruco
#include "aruco/aruco.h"
#include "aruco/markerdetector.h"

// opencv
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

// ros
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include "cv_bridge/cv_bridge.h"
#include "ros/package.h"
#include "detect_marker/Markers.h"

// standard
#include <iostream>
#include <stdio.h>





class DetectMarker
{
public:
    DetectMarker();
    void rgbCallback(const sensor_msgs::ImageConstPtr& msg);
    void Detection();
private:
    cv_bridge::CvImageConstPtr cv_ptr;
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub;
    detect_marker::Markers ui8_markers;

    void publishMarker();
};

#endif // DETECTMARKER_H
