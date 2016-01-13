#ifndef DETECTMARKER_H
#define DETECTMARKER_H
//aruco
#include "aruco/aruco.h"
#include "aruco/markerdetector.h"
#include "aruco/marker.h"
#include "aruco/cameraparameters.h"
#include "aruco/board.h"

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
#include "geometry_msgs/Vector3Stamped.h"

// standard
#include <iostream>
#include <stdio.h>





class DetectMarker
{
public:
    DetectMarker();
    void rgbCallback(const sensor_msgs::ImageConstPtr& msg);
    void vectorDetectCallback(const geometry_msgs::Vector3StampedPtr &msg);
    void Detection();
private:
    ros::Subscriber vectorSub;
    cv_bridge::CvImageConstPtr cv_ptr;
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Subscriber camInfoSub;
    ros::Publisher pub;
    detect_marker::Markers ui8_markers;
    unsigned int nextId;
    void publishMarker();

};

#endif // DETECTMARKER_H
