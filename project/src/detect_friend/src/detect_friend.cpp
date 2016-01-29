#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include "detect_friend.h"
#include <sensor_msgs/Image.h>
#include "cv_bridge/cv_bridge.h"
#include "detect_friend/Friend_id.h"
#include "detect_friend/FriendsInfos.h"
//#include "opencv2/nonfree/nonfree.hpp"

using namespace cv;

DetectFriend::DetectFriend(ros::NodeHandle& nodeHandle): m_nodeHandle(nodeHandle)
{
    ROS_INFO("Subscribing to camera image topic...");
    m_cameraSub = m_nodeHandle.subscribe("/camera/rgb/image_raw", 1, &DetectFriend::cameraSubCallback, this);
    ros::Rate loopRate(10);
    while (ros::ok() && m_cameraSub.getNumPublishers() <= 0)
    loopRate.sleep();
    star_image= cv::imread( "/.......dir",1);
    mushroom_image= cv::imread( "/.......dir",1);
    coin_image= cv::imread( "/.......dir",1);
    ROS_INFO("Creating friend's topic...");
    m_friend_idPub = m_nodeHandle.advertise<detect_friend::FriendsInfos>("/friendinfo", 10);
    ROS_INFO("Done, everything's ready.");
}



void DetectFriend::cameraSubCallback(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_INFO("Received image from camera.");

   Mat img;
   Mat gray_img;
    try
    {
        cv_bridge::CvImageConstPtr imgPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        if (imgPtr == NULL)
        {
            ROS_WARN("Received NULL image.");

        }
        img = imgPtr->image;
    }
    catch (cv_bridge::Exception& ex)
    {
        ROS_ERROR("cv_bridge exception: %s", ex.what());
        exit(-1);
    }
    
    cvtColor(img, gray_img, CV_BGR2GRAY);
    friends_inscene(gray_img, star_image, 1);
    friends_inscene(gray_img, mushroom_image, 2);
    friends_inscene(gray_img, coin_image, 3);

}



/** @function main */
 void DetectFriend::friends_inscene(const Mat &img_scene, const Mat &img_object, int j)
 {


  detect_friend::FriendsInfos friends;

  if( !img_scene.data)
  { std::cout<< " --(!) Error reading gray image from camera " ; }


  //-- Step 1: Detect the keypoints using SURF Detector
 // int minHessian = 400;
   ORB orb(500,1.2,9,25,0,2,ORB::HARRIS_SCORE, 25);
   std::vector<KeyPoint> keypoints_object, keypoints_scene;
   Mat descriptors_object, descriptors_scene;
   OrbDescriptorExtractor extractor;
   orb.detect(img_object, keypoints_object);
   orb.compute(img_object, keypoints_object, descriptors_object);
   orb.detect(img_scene, keypoints_scene);
   orb.compute(img_scene, keypoints_scene, descriptors_scene);

  BFMatcher matcher;
  std::vector< cv::DMatch > matches;
  matcher.match( descriptors_object, descriptors_scene, matches );

  double max_dist = 0; double min_dist = 100;
  
  //-- Quick calculation of max and min distances between keypoints
  for( int i = 0; i < descriptors_object.rows; i++ )
  { double dist = matches[i].distance;
    if( dist < min_dist ) min_dist = dist;
    if( dist > max_dist ) max_dist = dist;
  }
  printf("-a \n");
  printf("-- Max dist : %f \n", max_dist );
  printf("-- Min dist : %f \n", min_dist );
  printf("descriptors_object rows: %d /n",descriptors_object.rows); 

  //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
  std::vector<DMatch>good_matches;

  for( int i = 0; i < descriptors_object.rows; i++ )
  { if( matches[i].distance < 3*min_dist )
     { good_matches.push_back( matches[i]); }
  }
 
  Mat img_matches;
  drawMatches( img_object, keypoints_object, img_scene, keypoints_scene, good_matches, img_matches, Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );


  //-- Localize the object
  std::vector<Point2f> obj;
  std::vector<Point2f> scene;
 
  for( int i = 0; i < good_matches.size(); i++ )
  {
    //-- Get the keypoints from the good matches
    obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
    scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
  }
  printf("c \n");
  Mat H;
 
  if( !obj.empty() && !scene.empty() && good_matches.size() >= 4) {
  H = findHomography( obj, scene, cv::RANSAC );

  //-- Get the corners from the image_1 ( the object to be "detected" )
  std::vector<Point2f> obj_corners(4);
  obj_corners[0] = cvPoint(0,0); 
  obj_corners[1] = cvPoint( img_object.cols, 0 );
  obj_corners[2] = cvPoint( img_object.cols, img_object.rows ); 
  obj_corners[3] = cvPoint( 0, img_object.rows );
  std::vector<Point2f> scene_corners(4);
  printf("d \n");
  perspectiveTransform(obj_corners, scene_corners, H);

  //-- Draw lines between the corners (the mapped object in the scene - image_2 )
  line( img_matches, scene_corners[0] + cv::Point2f( img_object.cols, 0), scene_corners[1] + cv::Point2f( img_object.cols, 0), Scalar(0, 255, 0), 4 );
  line( img_matches, scene_corners[1] + cv::Point2f( img_object.cols, 0), scene_corners[2] + cv::Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
  line( img_matches, scene_corners[2] + cv::Point2f( img_object.cols, 0), scene_corners[3] + cv::Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
  line( img_matches, scene_corners[3] + cv::Point2f( img_object.cols, 0), scene_corners[0] + cv::Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
  friends.id=j;
  m_friend_idPub.publish(friends);
}

   printf("e \n");
  //-- Show detected matches
  Mat dst;
  resize(img_matches, dst, Size(480, 640), 0,0, INTER_LINEAR  );
  imshow( "Good Matches & Object detection", dst);
  printf("f \n");
  waitKey(0);

  }




void DetectFriend::Identification()
{
    ROS_INFO("Starting identification.");
    ros::spin();
}

