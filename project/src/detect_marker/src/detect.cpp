#include "detectmarker.h"

 /** @function main */
 int main( int argc, char** argv )
 {
     ros::init(argc, argv, "detect_marker");

     DetectMarker dm;

     ros::Rate	rate(100);
     while (ros::ok()){
             dm.Detection();
     }
 };
