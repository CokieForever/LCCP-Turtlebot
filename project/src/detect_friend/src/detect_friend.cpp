#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include "detect_friend.h"
#include <sensor_msgs/Image.h>
#include "cv_bridge/cv_bridge.h"
#include "detect_friend/Friend_id.h"
#include "detect_friend/FriendsInfos.h"
#include "orbdetector.h"
#include <sensor_msgs/image_encodings.h>


using namespace cv;
using namespace std;


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
    if( !star_image.data || !mushroom_image.data || !coin_image.data )
    {
        ROS_INFO("EMPTY IMAGE");
    }
    ROS_INFO("Creating friend's topic...");
    m_friend_idPub = m_nodeHandle.advertise<detect_friend::FriendsInfos>("/friendinfo", 10);
    ROS_INFO("Done, everything's ready.");
}



void DetectFriend::cameraSubCallback(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_INFO("Received image from camera.");
    detect_friend::FriendsInfos friends;
    detect_friend::Friend_id id_friend;
    Mat img;

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

      clusters(img, 0);
      clusters(img, 1);
      clusters(img, 2);

}

std::vector<Rect> DetectFriend::clusters(Mat img, int id)
{


  cv::Mat diff;
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
  vector<vector<Point> > contours_poly( contours.size() );
  vector<Rect> boundRect( contours.size() );
  Mat drawing = Mat::zeros( diff.size(), CV_8UC3 );
  RNG rng(12345);
  double area;
  int i, j;
  Scalar scalarstarmin(0,130,200);
  Scalar scalarstarmax(30,255,255);
  Scalar scalarmushmin(0,0,100);
  Scalar scalarmushmax(30,30,255);
  Scalar scalarcoinmin(0,210 ,250);
  Scalar scalarcoinmax(90,255,255);

  //select the approptiate scalar for the color detection according to the item's id
  if (id==0)
    {
      //set img element to 0 if its value is not in the range of scalars, otherwise to 1
      inRange(img, scalarstarmin, scalarstarmax, diff);
    }
  else if(id==1)
    {
      //set img element to 0 if its value is not in the range of scalars, otherwise to 1
      inRange(img, scalarmushmin, scalarmushmax, diff);
    }
  else if(id==2)
    {
      //set img element to 0 if its value is not in the range of scalars, otherwise to 1
      inRange(img, scalarcoinmin, scalarcoinmax, diff);
    }


  //find contours in the image.
  findContours( diff, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

  //ignore the black contours
  for(int i=0; i<contours.size(); i++)
   {
    if (contourArea(contours[i],true) > 0)
      {
       ROS_INFO("black");
       contours.erase(contours.begin() + i);
       hierarchy.erase(hierarchy.begin()+i);
       i--;
      }
    else
      {
        ROS_INFO("white");
      }
  }

  ROS_INFO("contour size %lu", contours.size() );


  //create rectangles around contours of interest and draw
  for( int i = 0; i < contours.size(); i++ )
    {
      approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
      area=contourArea(contours_poly[i]);
      ROS_INFO("area %.3f", area );
      //ignore the contours that have area less than 10*10
      if (area<100)
        {
         contours.erase(contours.begin() + i);
         hierarchy.erase(hierarchy.begin()+i);
         i--;
        }
      else
        {
         boundRect[i] = boundingRect( Mat(contours_poly[i]) );
         Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
         rectangle( img,   boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
         drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
         rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
        }
    }

 imshow( "Contours", drawing );
 imshow("final", img);
 waitKey(0);
 //return vector of rectangles
 return boundRect;
}

void DetectFriend::Identification()
{
    ROS_INFO("Starting identification.");
    ros::spin();
}

