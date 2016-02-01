#ifndef FRIEND_FINDER_HPP
#define FRIEND_FINDER_HPP

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "cv_bridge/cv_bridge.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Bool.h"

class FriendFinder
{
  public:
  
    FriendFinder();
    void setFriendDirectory();

  private:
    
    ros::NodeHandle m_node;
    ros::Subscriber m_imageSub;
    ros::Subscriber m_setImageCallbackSub;
    ros::Publisher  m_foundFriendPub;
    
    cv::Mat m_star;
    cv::Mat m_coin;
    cv::Mat m_mushroom;
    
    bool m_friendsReaded[3];
    bool m_callbackActive;
    vector<vector<Point> >[3] templateContours;
    void imageSubCallback(const sensor_msgs::ImageConstPtr& msg);
    void setImageSubscribing(std_msgs::Bool active);
    void friendPublish(int friendId);
    void preperateFriendTemplates();

  
};

#endif // FRIEND_FINDER_HPP
