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

/**
*@brief constructor
*@param nodeHanlde 	The main node handle
*/
DetectFriend::DetectFriend(ros::NodeHandle& nodeHandle): m_nodeHandle(nodeHandle)
{
    ROS_INFO("Subscribing to camera image topic...");
    m_cameraSub = m_nodeHandle.subscribe("/camera/rgb/image_raw", 1, &DetectFriend::cameraSubCallback, this);//subscribing to the camera
    ros::Rate loopRate(10);
    while (ros::ok() && m_cameraSub.getNumPublishers() <= 0)
        loopRate.sleep();

    std::string packagePath = "~";
    if (!m_nodeHandle.getParam("package_path", packagePath))
       ROS_WARN("The package path is not set, it will default to '~'.");
    
    star_image= cv::imread( packagePath + "/star-ref.png");//reading the star reference image
    mushroom_image= cv::imread(packagePath + "/mushroom-ref.png");//reading the mushroom reference image
    coin_image= cv::imread(packagePath + "/coin-ref.png"); //reading the coin reference image
    if( !star_image.data || !mushroom_image.data || !coin_image.data )
    {
        ROS_INFO("EMPTY IMAGE");
        ROS_INFO("%s", (packagePath + "/coin-ref.png").c_str());
    }
    ROS_INFO("Creating friend's topic...");
    m_friend_idPub = m_nodeHandle.advertise<detect_friend::FriendsInfos>("/friendinfo", 10); // publisher of friend information
    Scalar yellowstar(255, 255, 0); //declaration of scalar for star color
    Scalar redmushroom(255, 0, 0); //declaration of scalar for mushroom color
    Scalar yellowcoin(247,215,32); //declaration of scalar for coin color
    FriendMatcher::TemplateInfo star; // TempalteInfo is a structure of FriendMatcher class
    FriendMatcher::TemplateInfo mushroom;
    FriendMatcher::TemplateInfo coin;

    ROS_INFO("Set template infos");

    star.image=star_image; //setting the members of the data structure TemplateInfo for all the friends
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

/**
*@brief Callback function of the topic "/camera/rgb/image_raw". The topic triggers the execution
 * @param msg the received Image message
*/

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
      FriendMatcher::MatchResult result1= m_friendmatcher.match(img, 0); //friend matching
      FriendMatcher::MatchResult result2= m_friendmatcher.match(img, 1);
      FriendMatcher::MatchResult result3= m_friendmatcher.match(img, 2);

      if (result1.score>min_score) // compare the score of the matching result and if it is larger than the minimum defined score, publish the infos
      {
        int id=0;
        friend_details=publish_infos_of_friend(img, id, result1.boundingRect);
        friend_details.Time = msg->header.stamp;
        friendsInfos.infos.push_back(friend_details);
        //id_friend.center.x=center_of_friend(int i, cv::Rect rect);

    
      }

      if (result2.score>min_score)
      {
        int id=1;
        friend_details=publish_infos_of_friend(img, id, result2.boundingRect);
        friend_details.Time = msg->header.stamp;
        friendsInfos.infos.push_back(friend_details);

      }

      if (result3.score>min_score)
      {
         int id=2;
        friend_details=publish_infos_of_friend(img, id, result3.boundingRect);
        friend_details.Time = msg->header.stamp;
        friendsInfos.infos.push_back(friend_details);
 
      }
      if (friendsInfos.infos.size()>0)
        {
          m_friend_idPub.publish(friendsInfos);
        }

}


/**
*@brief publish information of friends
 * @param img cv::Mat recorded image
 * @param id  int id of friend
 * @param rectangular  Rect rectangle surrounding the friend contour
*/
detect_friend::Friend_id DetectFriend::publish_infos_of_friend(const cv::Mat& img, int id, Rect rectangle )
{
  Point center; // coordinates of friend center in the recorded image
  Point corners[4]; // coordinated of the 4 corners of the detected friend in the recorded image
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
      friendInfo.x = 2*(center.x /(double)width)-1; //set the coordinate x of friend's center
      friendInfo.y = 2*(center.y/(double)height)-1; //set the coordinate y of friend's center
      friendInfo.dz = FRIEND_SIZE * FRIEND_REF_DIST / rectangle.height; //set dz distance of robot
      friendInfo.dx = FRIEND_SIZE * (center.x-width/2) / rectangle.width; // set dx from image center
      friendInfo.dy = FRIEND_SIZE * (center.y-height/2) / rectangle.height;//set dy from image center
      friendInfo.d = sqrt(friendInfo.dy*friendInfo.dy + friendInfo.dx*friendInfo.dx + friendInfo.dz*friendInfo.dz); //set distance
      friendInfo.id = id; //set id of friend
      return friendInfo;

}
}

/**
*@brief compute center of friend in recorded image
 * @param linePoints1[2] Point, corners of friend rectangle
 * @param linePoints2[2] Point, corners of friend rectangle
 * @param isecPoint Point, center of friend
*/
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

/**
*@brief start the process of identification. Called by main function
*/
void DetectFriend::Identification()
{
    ROS_INFO("Starting identification.");
    ros::spin();
}
