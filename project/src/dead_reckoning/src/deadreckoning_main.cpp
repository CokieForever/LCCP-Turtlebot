#include "deadreckoning.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "deadreckoning");
    ros::NodeHandle node("~");
    ROS_INFO("Initialized ROS.");

    bool simulation = true;
    std::string mode;
    if (node.getParam("mode", mode))
        simulation = mode != "realworld";
    ROS_INFO("Mode: %s", simulation ? "Simulation" : "Real world");

    srand(time(0));
    
    double minX=-5, maxX=5, minY=-5, maxY=5;
    if (simulation)
    {
        minX = 0; maxX = 10;
        minY = 0; maxY = 10;
    }
    DeadReckoning dr(node, simulation, minX, maxX, minY, maxY);
    if (dr.ready())
        dr.reckon();
    
    ROS_INFO("Bye!");
    return 0;
};

