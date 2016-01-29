#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include "orbdetector.h"
#include "opencv2/highgui/highgui.hpp"

using namespace cv;

int main( int argc, char** argv )
{
    ros::init(argc, argv, "orb_test");
    ros::NodeHandle node("~");
    ROS_INFO("Initialized ROS.");
    
    std::string templatePath = "";
    if (!node.getParam("template_path", templatePath))
    {
        ROS_ERROR("The template path was not provided.");
        return 1;
    }
    
    std::string imagePath = "";
    if (!node.getParam("image_path", imagePath))
    {
        ROS_ERROR("The image path was not provided.");
        return 1;
    }
    
    Mat img_object = imread( templatePath, CV_LOAD_IMAGE_GRAYSCALE );
    Mat img_scene = imread( imagePath, CV_LOAD_IMAGE_GRAYSCALE );

    if( !img_object.data || !img_scene.data )
    {
        ROS_ERROR("Error while reading images (%s, %s)", imagePath.c_str(), templatePath.c_str());
        return 1;
    }

    ORBDetector detector;
    ORBDetector::ORBMatchResult result = detector.match(img_scene, img_object);
    if (result.score < 0)
    {
        ROS_INFO("No match found.");
        return 0;
    }
    
    ROS_INFO("Corner 1: %f, %f", result.corners[0].x, result.corners[0].y);
    ROS_INFO("Corner 2: %f, %f", result.corners[2].x, result.corners[2].y);
    ROS_INFO("Score: %.2f%%", result.score*100);
    
    Mat display = detector.drawResult(img_scene, result);
    imshow("Result", display);
    waitKey(0);
    
    return 0;
}