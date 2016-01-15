#include "initial_detection.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char **argv){
	
    ros::init(argc, argv, "initial_detection");
    initial_detection initial_detections;
  

    ros::Rate rate(100);
    
   
    while(ros::ok())
    {
        initial_detections.startRotating();
        rate.sleep();
    }
	return 0;
	
};
