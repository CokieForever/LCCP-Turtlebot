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
    detect_friend::FriendsInfos friendsInfos;
    detect_friend::Friend_id friend_details;
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
        int id=0;
        friend_details=publish_infos_of_friend(img, id, result1.boundingRect);
        friendsInfos.infos.push_back(friend_details);
        //id_friend.center.x=center_of_friend(int i, cv::Rect rect);

        /*cv::Mat display = m_friendmatcher.drawResult(img, result1);
        cv::imshow("Result1", display);
        cv::waitKey(1);*/
      }

      if (result2.score>min_score)
      {
        int id=1;
        friend_details=publish_infos_of_friend(img, id, result2.boundingRect);
        friendsInfos.infos.push_back(friend_details);
        /*cv::Mat display = m_friendmatcher.drawResult(img, result2);
        cv::imshow("Result2", display);
        cv::waitKey(1);*/
      }

      if (result3.score>min_score)
      {
         int id=2;
        friend_details=publish_infos_of_friend(img, id, result3.boundingRect);
        friendsInfos.infos.push_back(friend_details);
        /*cv::Mat display = m_friendmatcher.drawResult(img, result3);
        cv::imshow("Result3", display);
        cv::waitKey(1);*/
      }
      if (friendsInfos.infos.size()>0)
        {
          m_friend_idPub.publish(friendsInfos);
        }

}


detect_friend::Friend_id DetectFriend::publish_infos_of_friend(const cv::Mat& img, int id, Rect rectangle )
{
  Point center;
  Point corners[4];
  double width=img.rows;
  double height=img.cols;
  corners[1].x=rectangle.tl().x;
  corners[1].y=rectangle.tl().y;
  corners[2].x=rectangle.tl().x+rectangle.width;
  corners[2].y=rectangle.tl().y;
  corners[3].x=rectangle.br().x;
  corners[3].y=rectangle.br().y;
  corners[4].x=rectangle.tl().x;
  corners[4].y=rectangle.tl().y-rectangle.height;
  double FRIEND_SIZE;


  if (!ComputeQuadrilateralCenter(corners, &center))
      ROS_WARN("Unable to compute center.");
  else
  {
      if (id==0)
        {
          FRIEND_SIZE=STAR_HEIGHT;
        }
      else if (id==1)
        {
          FRIEND_SIZE=MUSHROOM_HEIGHT;
        }
      else if(id==2)
        {
          FRIEND_SIZE=COIN_HEIGHT;
        }

      detect_friend::Friend_id friendInfo;
      friendInfo.x = 2*(center.x /(double)width)-1;
      friendInfo.y = 2*(center.y/(double)height)-1;
      friendInfo.dz = FRIEND_SIZE * FRIEND_REF_DIST / rectangle.height;
      friendInfo.dx = FRIEND_SIZE * (center.x-width/2) / rectangle.width;
      friendInfo.dy = FRIEND_SIZE * (center.y-height/2) / rectangle.height;
      friendInfo.d = sqrt(friendInfo.dy*friendInfo.dy + friendInfo.dx*friendInfo.dx + friendInfo.dz*friendInfo.dz);
      friendInfo.id = id;
      friendInfo.Time=ros::Time::now();
      return friendInfo;

}
}

 bool DetectFriend::ComputeLinesIntersection(Point linePoints1[2], Point linePoints2[2], Point *isectPoint)
  {
      double x1 = linePoints1[0].x;
      double x2 = linePoints1[1].x;
      double y1 = linePoints1[0].y;
      double y2 = linePoints1[1].y;

      double x3 = linePoints2[0].x;
      double x4 = linePoints2[1].x;
      double y3 = linePoints2[0].y;
      double y4 = linePoints2[1].y;

      double d = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4);
      if (fabs(d) < 1e-6)
          return false;

      isectPoint->x = ((x1*y2 - y1*x2) * (x3-x4) - (x1-x2) * (x3*y4 - y3*x4)) / d;
      isectPoint->y = ((x1*y2 - y1*x2) * (y3-y4) - (y1-y2) * (x3*y4 - y3*x4)) / d;
      return true;
  }

  bool DetectFriend::ComputeQuadrilateralCenter(Point points[4], Point *centerPoint)
  {
      Point linePoints1[2] = {points[0], points[2]};
      Point linePoints2[2] = {points[1], points[3]};
      return ComputeLinesIntersection(linePoints1, linePoints2, centerPoint);
  }


void DetectFriend::Identification()
{
    ROS_INFO("Starting identification.");
    ros::spin();
}
