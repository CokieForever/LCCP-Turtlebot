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
#include "orbdetector.h"
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
    detect_friend::FriendsInfos friends;
    detect_friend::Friend_id id_friend;
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
    ORBDetector orbdetector;
    cvtColor(img, gray_img, CV_BGR2GRAY);
    ORBDetector::ORBMatchResult result1= orbdetector.match(gray_img, star_image);
    ORBDetector::ORBMatchResult result2= orbdetector.match(gray_img, mushroom_image);
    ORBDetector::ORBMatchResult result3= orbdetector.match(gray_img, coin_image);
    if (result1.score>min_score)
    {
      id_friend.id=0;
      friends.infos.push_back(id_friend);
    }
 
     if (result2.score>min_score)
    {
      id_friend.id=1;
      friends.infos.push_back(id_friend);
    }
    
    if (result3.score>min_score)
    {
      id_friend.id=2;
      friends.infos.push_back(id_friend);
    }
    m_friend_idPub.publish(friends);

}



void DetectFriend::Identification()
{
    ROS_INFO("Starting identification.");
    ros::spin();
}

