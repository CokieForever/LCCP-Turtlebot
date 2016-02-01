#ifndef SOUND_NODE_HPP
#define SOUND_NODE_HPP
#include "ros/ros.h"
#include <iostream>
#include <stdio.h>
#include "std_msgs/Int32.h"
#include "std_msgs/Empty.h"
#include <stdlib.h>
#include <std_msgs/Bool.h>
//#include "detect_friend/Friendinfos"

class SoundClass
{
	public:
		
		SoundClass();

		void checkStarTime();

	private:
    
		ros::NodeHandle m_rosNode;
		//ros::Subscriber m_rosFriendIdSub;
		ros::Subscriber m_rosBangedBoxSub;
		ros::Publisher m_rosPowerUp;
		
	//	void getFriendIdCallback(const detect_friend::FriendsInfos &friendInfo);
		std::string getFriendName(int id);

		void bangedBoxCallback(const std_msgs::EmptyConstPtr &empty);
		void playSoundForFriend(std::string friendName);
		int generateRandomFriend();
		ros::Time m_startTime;
		std_msgs::Bool m_isPowered;
		bool m_bfoundedFriends[3];
};
#endif
