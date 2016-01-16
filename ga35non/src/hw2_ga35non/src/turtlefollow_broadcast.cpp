#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>
#include <turtlesim/Spawn.h>
#include <vector>
#include <std_srvs/Empty.h>

#include "utilities.h"

class Broadcaster
{
	private:
		tf::TransformBroadcaster m_broadcaster;
		std::string m_turtleName;
		ros::Subscriber m_turtleSubscriber;
		
		void updateTurtleTransform(const turtlesim::Pose::ConstPtr& msg)
		{
			tf::Transform transform;
			tf::Quaternion q;
			
			transform.setOrigin(tf::Vector3(msg->x, msg->y, 0.0));
			q.setRPY(0, 0, msg->theta);
			transform.setRotation(q);
			m_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", m_turtleName));
		}

	public:
		Broadcaster(const std::string& turtleName, ros::NodeHandle& node): m_turtleName(turtleName)
		{
			ROS_INFO("Subscribing to turtle \"%s\"...", m_turtleName.c_str());
			m_turtleSubscriber = node.subscribe("/"+m_turtleName+"/pose", 10, &Broadcaster::updateTurtleTransform, this);
			ros::Rate rate(10);
			while (ros::ok() && m_turtleSubscriber.getNumPublishers() <= 0)
				rate.sleep();
			checkRosOk_v();
		}

		std::string getTurtleName() const
		{
			return m_turtleName;
		}
};

class MyLittleTurtlesBroadcast
{
	private:
		std::vector<Broadcaster*> m_turtleBroadcasters;
		ros::NodeHandle& m_node;

	public:
		MyLittleTurtlesBroadcast(ros::NodeHandle& node, const std::vector<std::string>& turtleNames = std::vector<std::string>()): m_node(node)
		{
			ROS_INFO("Reseting Turtlesim...");
			ros::service::waitForService("reset");
			checkRosOk_v();
			
			ros::ServiceClient resetService = m_node.serviceClient<std_srvs::Empty>("reset");
			std_srvs::Empty empty;
			resetService.call(empty);

			for (std::vector<std::string>::const_iterator it = turtleNames.begin() ; it != turtleNames.end(); it++)
			{
				if (*it == "turtle1")
				{
					Broadcaster* br = new Broadcaster(*it, m_node);
					m_turtleBroadcasters.push_back(br);
				}
				else
					addTurtle(*it);
			}
		}

		~MyLittleTurtlesBroadcast()
		{
			for (std::vector<Broadcaster*>::iterator it = m_turtleBroadcasters.begin() ; it != m_turtleBroadcasters.end(); it++)
			{
				if ((*it) != NULL)
					delete *it;
			}
		}

		bool addTurtle(const std::string& turtleName)
		{
			ROS_INFO("Adding turtle \"%s\"...", turtleName.c_str());
			
			for (std::vector<Broadcaster*>::iterator it = m_turtleBroadcasters.begin() ; it != m_turtleBroadcasters.end(); it++)
			{
				if ((*it)->getTurtleName() == turtleName)
				{
					ROS_INFO("Turtle \"%s\" is already registered.", turtleName.c_str());
					return false;
				}
			}
			
			ros::service::waitForService("spawn");
			checkRosOk(false);

			ros::ServiceClient addTurtleService = m_node.serviceClient<turtlesim::Spawn>("spawn");
			turtlesim::Spawn spawn;
			spawn.request.x = (std::rand() / (double)RAND_MAX) * 10.0 + 0.5;
			spawn.request.y = (std::rand() / (double)RAND_MAX) * 10.0 + 0.5;
			spawn.request.theta = 0.0;
			spawn.request.name = turtleName;
			if (!addTurtleService.call(spawn))
			{
				ROS_WARN("Unable to create turtle \"%s\".", turtleName.c_str());
				return false;
			}

			Broadcaster* broadcaster = new Broadcaster(turtleName, m_node);
			m_turtleBroadcasters.push_back(broadcaster);

			return true;
		}

		void broadcast()
		{
			ROS_INFO("Starting broadcast.");
			ros::spin();
		}
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "turtlefollow_broadcast");
	ros::NodeHandle node;
	ROS_INFO("Initialized ROS.");

	std::srand(std::time(0));
	
	std::vector<std::string> turtleNames;
	for (int i=1 ; i < argc ; i++)
		turtleNames.push_back(argv[i]);
	if (turtleNames.size() <= 0)
		turtleNames.push_back("turtle1");

	MyLittleTurtlesBroadcast turtlesBroadcast(node, turtleNames);
	turtlesBroadcast.broadcast();

	ROS_INFO("Bye!");
	return 0;
}


