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

    std::string packagePath = "~";
    if (!m_nodeHandle.getParam("package_path", packagePath))
       ROS_WARN("The package path is not set, it will default to '~'.");
    
    star_image= cv::imread( packagePath + "/star-ref.png");
    mushroom_image= cv::imread(packagePath + "/mushroom-ref.png");
    coin_image= cv::imread(packagePath + "/coin-ref.png");
    if( !star_image.data || !mushroom_image.data || !coin_image.data )
    {
        ROS_INFO("EMPTY IMAGE");
        ROS_INFO("%s", (packagePath + "/coin-ref.png").c_str());
    }
    ROS_INFO("Creating friend's topic...");
    m_friend_idPub = m_nodeHandle.advertise<detect_friend::FriendsInfos>("/friendinfo", 10);
    Scalar yellowstar(255, 255, 0);
    Scalar redmushroom(255, 0, 0);
    Scalar yellowcoin(247,215,32);
    FriendMatcher::TemplateInfo star;
    FriendMatcher::TemplateInfo mushroom;
    FriendMatcher::TemplateInfo coin;

    ROS_INFO("Set template infos");

    star.image=star_image;
    star.id=0;
    star.mainColor=yellowstar;
    star.name="Star";
    star.w=0.2;
    star.h=0.2134;
    star.roi.x=30;
    star.roi.y=30;
    star.roi.width=390;
    star.roi.height=420;
    m_friendmatcher.addTemplate(star);

    mushroom.image=mushroom_image;
    mushroom.id=1;
    mushroom.mainColor=redmushroom;
    mushroom.name="Mushroom";
    mushroom.w=0.2;
    mushroom.h=0.2;
    mushroom.roi.x=30;
    mushroom.roi.y=30;
    mushroom.roi.width=420;
    mushroom.roi.height=300;
    m_friendmatcher.addTemplate(mushroom);

    coin.image=coin_image;
    coin.id=2;
    coin.mainColor=yellowstar;
    coin.name="Coin";
    coin.w=0.2;
    coin.h=0.27;
    coin.roi.x=60;
    coin.roi.y=60;
    coin.roi.width=270;
    coin.roi.height=390;
    m_friendmatcher.addTemplate(coin);

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

     //objects ids: 0 for star, 1 for mushroom, 2 for coin
      FriendMatcher::MatchResult result1= m_friendmatcher.match(img, 0);
      FriendMatcher::MatchResult result2= m_friendmatcher.match(img, 1);
      FriendMatcher::MatchResult result3= m_friendmatcher.match(img, 2);

      if (result1.score>min_score)
      {
        id_friend.id=0;
        friends.infos.push_back(id_friend);
        /*cv::Mat display = m_friendmatcher.drawResult(img, result1);
        cv::imshow("Result1", display);
        cv::waitKey(1);*/
      }

      if (result2.score>min_score)
      {
        id_friend.id=1;
        friends.infos.push_back(id_friend);
        /*cv::Mat display = m_friendmatcher.drawResult(img, result2);
        cv::imshow("Result2", display);
        cv::waitKey(1);*/
      }

      if (result3.score>min_score)
      {
        id_friend.id=2;
        friends.infos.push_back(id_friend);
        /*cv::Mat display = m_friendmatcher.drawResult(img, result3);
        cv::imshow("Result3", display);
        cv::waitKey(1);*/
      }
        
      m_friend_idPub.publish(friends);

      




}

void DetectFriend::Identification()
{
    ROS_INFO("Starting identification.");
    ros::spin();
}
