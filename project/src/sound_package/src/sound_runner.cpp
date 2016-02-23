#include "sound_node.hpp"


int main( int argc, char **argv)
	{
		ros::init(argc, argv, "Sound");
		SoundClass sound;
		
		while(ros::ok())
		{
			sound.checkStarTime();
			ros::spinOnce();
		}
		return 0;
	}	
