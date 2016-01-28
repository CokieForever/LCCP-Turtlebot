#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "cv_bridge/cv_bridge.h"

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

    //-- Step 1: Detect the keypoints using ORB Detector
    ORB orb(500, 1.2, 9, 25, 0, 2, ORB::HARRIS_SCORE, 25);

    std::vector<KeyPoint> keypoints_object, keypoints_scene;

    ROS_INFO("Detection");
    orb.detect( img_object, keypoints_object );
    orb.detect( img_scene, keypoints_scene );

    //-- Step 2: Calculate descriptors (feature vectors)
    Mat descriptors_object, descriptors_scene;

    ROS_INFO("Extraction");
    orb.compute( img_object, keypoints_object, descriptors_object );
    orb.compute( img_scene, keypoints_scene, descriptors_scene );

    //-- Step 3: Matching descriptor vectors using FLANN matcher
    ROS_INFO("Matching");
    BFMatcher matcher(NORM_HAMMING);
    std::vector< DMatch > matches;
    matcher.match( descriptors_object, descriptors_scene, matches );

    double max_dist = 0; double min_dist = 100;

    //-- Quick calculation of max and min distances between keypoints
    for( int i = 0; i < descriptors_object.rows; i++ )
    {
        double dist = matches[i].distance;
        if( dist < min_dist )
            min_dist = dist;
        if( dist > max_dist )
            max_dist = dist;
    }

    ROS_INFO("Max dist : %f", max_dist );
    ROS_INFO("Min dist : %f", min_dist );

    //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
    std::vector< DMatch > good_matches;

    for( int i = 0; i < descriptors_object.rows; i++ )
    {
        if( matches[i].distance < 3*min_dist )
        {
            good_matches.push_back( matches[i]);
        }
    }

    Mat img_matches;
    drawMatches( img_object, keypoints_object, img_scene, keypoints_scene,
        good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
        vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
    
    //-- Localize the object
    std::vector<Point2f> obj;
    std::vector<Point2f> scene;

    for( int i = 0; i < good_matches.size(); i++ )
    {
        //-- Get the keypoints from the good matches
        obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
        scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
    }
    ROS_INFO("Localization");
    Mat H = findHomography( obj, scene, CV_RANSAC );

    //-- Get the corners from the image_1 ( the object to be "detected" )
    std::vector<Point2f> obj_corners(4);
    obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( img_object.cols, 0 );
    obj_corners[2] = cvPoint( img_object.cols, img_object.rows ); obj_corners[3] = cvPoint( 0, img_object.rows );
    std::vector<Point2f> scene_corners(4);

    perspectiveTransform( obj_corners, scene_corners, H);
    ROS_INFO("Corner 1: %f, %f", scene_corners[0].x, scene_corners[0].y);
    ROS_INFO("Corner 2: %f, %f", scene_corners[2].x, scene_corners[2].y);
    
    //-- Draw lines between the corners (the mapped object in the scene - image_2 )
    line( img_matches, scene_corners[0] + Point2f( img_object.cols, 0), scene_corners[1] + Point2f( img_object.cols, 0), Scalar(0, 255, 0), 4 );
    line( img_matches, scene_corners[1] + Point2f( img_object.cols, 0), scene_corners[2] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
    line( img_matches, scene_corners[2] + Point2f( img_object.cols, 0), scene_corners[3] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
    line( img_matches, scene_corners[3] + Point2f( img_object.cols, 0), scene_corners[0] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );

    //-- Show detected matches
    imshow( "Good Matches & Object detection", img_matches );

    waitKey(0);
    return 0;
}