#ifndef SUPER_MARIO_HPP
#define SUPER_MARIO_HPP
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "cv_bridge/cv_bridge.h"
#include <iostream>
#include <stdio.h>
#include "std_msgs/Bool.h"


class SuperMario
{
public:
  SuperMario(ros::NodeHandle &nh, cv::Mat &tmpl);
  void Start();
private:
  void imageCallback(const sensor_msgs::ImageConstPtr &img_msg);
  void MatchingMethod( int, void* );
  void movementCallback(std_msgs::Bool move_bool);

  ros::NodeHandle m_nh;
  ros::Subscriber m_imageSub;
  ros::Subscriber m_movementSub;


  cv::Mat m_img; cv::Mat m_templ; cv::Mat m_result; cv::Mat m_newResult;
  int m_match_method;
  cv::Mat m_check_img;
  bool m_checkcoin;


};


#endif
