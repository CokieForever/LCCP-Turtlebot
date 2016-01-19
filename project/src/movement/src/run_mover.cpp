#include "movement.hpp"

int main(int argc, char **argv){
	
	ros::init(argc, argv, "stopper");
	Mover mover;

	
    ros::Rate rate(100);
    while(ros::ok())
    {
        mover.startMoving();
    }
	return 0;
	
};
