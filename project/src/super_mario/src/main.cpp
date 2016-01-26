#include "super_mario.hpp"

using namespace std;
using namespace cv;


/** @function main */
int main( int argc, char** argv )
{
  ros::init(argc, argv, "super_mario");
  ros::NodeHandle nh;
  cv::Mat templ;
  /// Load image and template
  templ = imread( "/home/alex/Desktop/screenshots/coin-us-dollar-icon.png", 1 );
  ROS_INFO("Read image");

  SuperMario sm(nh,templ);
  sm.Start();

}

/**
 * @function MatchingMethod
 * @brief Trackbar callback
 */
