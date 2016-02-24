#include "sound_node.hpp"
#include <time.h>
#include <stdio.h>


SoundClass::SoundClass()
{
    m_rosFriendIdSub = m_rosNode.subscribe("/friendinfo", 5, &SoundClass::getFriendIdCallback, this);
    m_rosBangedBoxSub = m_rosNode.subscribe("/bangedBox", 5, &SoundClass::bangedBoxCallback, this);
    m_rosPowerUp = m_rosNode.advertise<std_msgs::Bool>("gotStar", 10);
    m_startTime = ros::Time::now();
    m_timer = ros::Time::now();
    m_bfoundedFriends[0] = false;
    m_bfoundedFriends[1] = false;
    m_bfoundedFriends[2] = false;
    m_isPowered.data = 0;
}


void SoundClass::getFriendIdCallback(const detect_friend::FriendsInfos &friendInfo)
{
	ROS_INFO("In getFriendIdCallback");
	if(friendInfo.infos.size() && (ros::Time::now()-m_timer).toSec()>3.0)
		{

			for(int i = 0; i<friendInfo.infos.size(); i++)
				{
					std::cout<<"found friend with ID: "<<friendInfo.infos[i].id<<std::endl;

					std::string buffName = getFriendName(friendInfo.infos[i].id);
					std::cout<<buffName<<std::endl;
					if(buffName=="star" && !m_isPowered.data)
						{
							m_startTime = ros::Time::now();
							m_isPowered.data = 1;
							m_rosPowerUp.publish(m_isPowered);
							m_isPowered.data = 0;
							std::cout<<"Power Up!"<<std::endl;
						}
					else if(buffName=="star" && m_isPowered.data)
						{
							m_startTime = ros::Time::now();
							std::cout<<"Power Up again!"<<std::endl;
						}

							playSoundForFriend(buffName);
							sleep(1);
				}
		}
	else{ std::cout<<"Sorry, got no friends :-( "<<std::endl;}
}


void SoundClass::playSoundForFriend(std::string friendName)
{
	if(system(NULL))
	{
			m_timer = ros::Time::now();
		if(friendName=="star")
			{ if( m_isPowered.data==0)
					{
						m_isPowered.data = 1;
						system("aplay --duration=15 ~/group7/project/src/media_source/audio/star.wav &" );
						m_bfoundedFriends[0]=true;
					}
				else{system("pkill -9 aplay && aplay --duration=15 ~/group7/project/src/media_source/audio/star.wav &" ); m_bfoundedFriends[0]=true;}

			}

		else if(friendName=="coin")			{ system("aplay ~/group7/project/src/media_source/audio/coin.wav & "); m_bfoundedFriends[1]=true;}

		else if(friendName=="mushroom")	{ system("aplay ~/group7/project/src/media_source/audio/mushroom.wav &"); m_bfoundedFriends[2]=true;}

		else														{ std::cerr<<"Error: unknown friend"<<std::endl; }
	}	
}
	
int SoundClass::generateRandomFriend()
{
	srand (time(NULL));

	int returnValue= -1;
	double randNum =  ((double) rand() / (RAND_MAX));
	
	if(randNum<=0.1){returnValue = 1;}
	else if(randNum>0.1 && randNum <= 0.85){returnValue = 0;}
	else if(randNum>0.85){returnValue = 2;}
	

return returnValue;
}
	
std::string SoundClass::getFriendName(int id)
{
	std::string buffer;
	std::string star = "star";
	std::string coin = "coin";
	std::string mushroom = "mushroom";

	
	switch(id)
	{
		case 0: buffer = star; break;
		case 1: buffer = mushroom; break;
		case 2: buffer = coin; break;
		default: buffer = "empty"; break;
	}	
	return buffer;
}	
	
void SoundClass::bangedBoxCallback(const std_msgs::EmptyConstPtr &empty)
{
	ROS_INFO("In bangedBox");
	int randomFriend = generateRandomFriend();
	if(randomFriend!=-1)
		{
			std::string buffName = getFriendName(randomFriend);
			std::cout<<buffName<<std::endl;
			if(buffName=="star" && !m_isPowered.data)
				{
					m_startTime = ros::Time::now();
					m_isPowered.data = 1;
					m_rosPowerUp.publish(m_isPowered);
					m_isPowered.data = 0;
					std::cout<<"Power Up!"<<std::endl;
				}
			else if(buffName=="star" && m_isPowered.data)
				{
					m_startTime = ros::Time::now();
					std::cout<<"Power Up again!"<<std::endl;
				}

			playSoundForFriend(buffName);

		}
	else{ std::cout<<"Error while generating friend name"<<std::endl;}
}	

void SoundClass::checkStarTime()
{
	double runnedTime=(ros::Time::now() - m_startTime).toSec();
	if(m_isPowered.data && runnedTime>15)
		{
			m_isPowered.data = 0;
			m_rosPowerUp.publish(m_isPowered);
			std::cout<<"no power anymore"<<std::endl;
		}
}


	
	
