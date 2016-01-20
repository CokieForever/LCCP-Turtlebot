#include "stopper.hpp"


int main(int argc, char **argv){
	
	ros::init(argc, argv, "stopper");
	Stopper stopper;

	
    ros::Rate rate(100);
    while(ros::ok())
    {
        stopper.startMoving();
    }
	return 0;
	
};
