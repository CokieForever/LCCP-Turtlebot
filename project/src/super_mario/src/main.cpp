#include "super_mario.hpp"

using namespace std;
using namespace cv;


/** @function main */
int main( int argc, char** argv )
{
  ros::init(argc, argv, "super_mario");
  ros::NodeHandle nh;
  cv::Mat coin;
  cv::Mat mushroom;
  cv::Mat star;
  /// Load image and template
  coin = imread( "/home/ga73kec/Pictures/LCPP/templates/coin.png", 1 );
  mushroom = imread( "/home/ga73kec/Pictures/LCPP/templates/mushroom.png", 1 );
  star = imread( "/home/ga73kec/Pictures/LCPP/templates/star.png", 1 );
  ROS_INFO("Read image");

  SuperMario sm(nh,coin, mushroom, star);
  sm.Start();

}

/**
 * @function MatchingMethod
 * @brief Trackbar callback
 */
