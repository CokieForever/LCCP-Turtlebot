#include "Stopper.h"

int main(int argc, char **argv){

	//Initiate new ROS node named "stopper"
	ros::init(argc, argv, "stopper");
	
	//Create new stopper objet
	Stopper stopper;
	
	//Start the movement
	stopper.startMoving();
	
	return 0;
};
