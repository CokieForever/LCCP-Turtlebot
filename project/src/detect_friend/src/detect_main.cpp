#include "detect_friend.h"

/** @function main */
int main( int argc, char** argv )
{
    ros::init(argc, argv, "detect_friend");
    ros::NodeHandle nodeHandle("~");
    ROS_INFO("Initialized ROS.");

    DetectFriend df(nodeHandle);
    df.Identification();
}
