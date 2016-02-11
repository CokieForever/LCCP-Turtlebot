#include "movement.hpp"
/**
 * @brief Main function of the mover node. Instantiates an object of the Mover class and starts the movement.
 * @param argc integer
 * @param argv char
 * @return 0
 */
int main(int argc, char **argv){
    ros::init(argc, argv, "movement");
	Mover mover;
    ros::Rate rate(100);
    while(ros::ok())
    {
        mover.startMoving();
    }
	return 0;
};
