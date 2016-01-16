//OpenCV
#include "opencv2/objdetect/objdetect.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "ros/package.h"
#include <iostream>
#include <stdio.h>

using namespace std;
using namespace cv;

void imageCallback(const sensor_msgs::ImageConstPtr& msg);
void detectAndDisplay(Mat frame);
/** Global variables */
 String face_cascade_name =  ros::package::getPath("face_detection") + "/src/haarcascade_frontalface_alt.xml";
 String eyes_cascade_name =  ros::package::getPath("face_detection") + "/src/haarcascade_eye_tree_eyeglasses.xml";
 CascadeClassifier face_cascade;
 CascadeClassifier eyes_cascade;
 RNG rng(12345);



int main(int argc, char **argv) {

  //Mat frame;

     //-- 1. Load the cascades

ros::init(argc, argv, "image_listener");
ros::NodeHandle nh;

image_transport::ImageTransport it(nh);
image_transport::Subscriber sub = it.subscribe("camera/rgb/image_rect_color", 1, imageCallback);
if( !face_cascade.load( face_cascade_name ) ){ printf("--(!)Error loading\n"); return -1; };
if( !eyes_cascade.load( eyes_cascade_name ) ){ printf("--(!)Error loading\n"); return -1; };
ros::spin();

}


void imageCallback(const sensor_msgs::ImageConstPtr& msg)

 {
  cv_bridge::CvImageConstPtr	cvImagePtr;
   try
   {
    cvImagePtr	=	cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
   // cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::waitKey(30);
    }
   catch (cv_bridge::Exception& e)
   {
     ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
     exit(-1);
   }
  Mat frame;
  if( cvImagePtr )
  {
   //while( true )
   //{
  frame = cvImagePtr->image;

 //-- 3. Apply the classifier to the frame
    if( !frame.empty() )
     { detectAndDisplay( frame ); }

 }
 }


void detectAndDisplay(Mat frame )
{
  std::vector<Rect> faces;
  Mat frame_gray;

  cvtColor( frame, frame_gray, CV_BGR2GRAY );
  equalizeHist( frame_gray, frame_gray );

  //-- Detect faces
  face_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );

  for( size_t i = 0; i < faces.size(); i++ )
  {
    Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
    ellipse( frame, center, Size( faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0 );

    Mat faceROI = frame_gray( faces[i] );
    std::vector<Rect> eyes;

    //-- In each face, detect eyes
    eyes_cascade.detectMultiScale( faceROI, eyes, 1.1, 2, 0 |CV_HAAR_SCALE_IMAGE, Size(30, 30) );

    for( size_t j = 0; j < eyes.size(); j++ )
     {
       Point center( faces[i].x + eyes[j].x + eyes[j].width*0.5, faces[i].y + eyes[j].y + eyes[j].height*0.5 );
       int radius = cvRound( (eyes[j].width + eyes[j].height)*0.25 );
       circle( frame, center, radius, Scalar( 255, 0, 0 ), 4, 8, 0 );
     }
  }
  //-- Show what you got
  cv::imshow("view", frame);
 }

