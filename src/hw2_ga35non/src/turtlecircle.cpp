#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/Pose.h>
#include <std_srvs/Empty.h>

#include "utilities.h"

class MyLittleTurtles
{
	private:
		static const double MARGIN_ERR = 0.05;
		
		static void updateTurtleStatus(geometry_msgs::Vector3& status, const turtlesim::Pose::ConstPtr& message)
		{
			status.x = message->x;
			status.y = message->y;
			status.z = message->theta;
		}

		static double vectorDist(geometry_msgs::Vector3 vec1, geometry_msgs::Vector3 vec2)
		{
			return (vec1.x-vec2.x)*(vec1.x-vec2.x) + (vec1.y-vec2.y)*(vec1.y-vec2.y);
		}

		ros::NodeHandle& m_nodeHandle;
		ros::Publisher m_ourTurtlePub;
		ros::Publisher m_theirTurtlePub;
		ros::Subscriber m_ourTurtleSub;
		ros::Subscriber m_theirTurtleSub;
		geometry_msgs::Vector3 m_ourStatus;
		geometry_msgs::Vector3 m_theirStatus;

		void updateOurTurtleStatus(const turtlesim::Pose::ConstPtr& message)
		{
			updateTurtleStatus(m_ourStatus, message);
		}

		void updateTheirTurtleStatus(const turtlesim::Pose::ConstPtr& message)
		{
			updateTurtleStatus(m_theirStatus, message);
		}
		
		void sendMoveOrder(double linear, double angular)
		{
			geometry_msgs::Twist twist;
			memset(&twist, 0, sizeof(geometry_msgs::Twist));
			twist.linear.x = linear;
			twist.angular.z = angular;
			m_ourTurtlePub.publish(twist);
			m_theirTurtlePub.publish(twist);
		}
	
	public:
		MyLittleTurtles(ros::NodeHandle& nodeHandle): m_nodeHandle(nodeHandle)
		{
			ROS_INFO("Reseting Turtlesim...");
			ros::service::waitForService("reset");
			ros::ServiceClient resetService = m_nodeHandle.serviceClient<std_srvs::Empty>("reset");
			std_srvs::Empty empty;
			resetService.call(empty);

			ROS_INFO("Adding a new turtle...");
			ros::service::waitForService("spawn");
			ros::ServiceClient addTurtleService = m_nodeHandle.serviceClient<turtlesim::Spawn>("spawn");
			turtlesim::Spawn spawn;
			spawn.request.x = 2.0;
			spawn.request.y = 4.0;
			spawn.request.theta = 0.0;
			spawn.request.name = "my_turtle";
			addTurtleService.call(spawn);

			std::string turtleName = spawn.response.name;
			ROS_INFO("Turtle created with name \"%s\".", turtleName.c_str());

			ROS_INFO("Creating publishers for our turtles...");
			m_ourTurtlePub = m_nodeHandle.advertise<geometry_msgs::Twist>("/"+turtleName+"/cmd_vel", 10);
			m_theirTurtlePub = m_nodeHandle.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
			ros::Rate rate(10);
			while (ros::ok() && (m_ourTurtlePub.getNumSubscribers() <= 0 || m_theirTurtlePub.getNumSubscribers() <= 0))
				rate.sleep();
			checkRosOk_v();
			
			ROS_INFO("Creating subscribers for our turtles...");
			m_ourTurtleSub = m_nodeHandle.subscribe("/"+turtleName+"/pose", 10, &MyLittleTurtles::updateOurTurtleStatus, this);
			m_theirTurtleSub = m_nodeHandle.subscribe("/turtle1/pose", 10, &MyLittleTurtles::updateTheirTurtleStatus, this);
			while (ros::ok() && (m_ourTurtleSub.getNumPublishers() <= 0 || m_theirTurtleSub.getNumPublishers() <= 0))
				rate.sleep();
			checkRosOk_v();
			
			ROS_INFO("Retrieving initial statuses...");
			ros::spinOnce();
			ROS_INFO("Our initial status: x=%.3f, y=%.3f, z=%.3f", m_ourStatus.x, m_ourStatus.y, m_ourStatus.z);
			ROS_INFO("Their initial status: x=%.3f, y=%.3f, z=%.3f", m_theirStatus.x, m_theirStatus.y, m_theirStatus.z);
			
			ROS_INFO("Everything's ready.");
		}

		bool circle()
		{
			geometry_msgs::Vector3 ourInitialStatus = m_ourStatus;
			geometry_msgs::Vector3 theirInitialStatus = m_theirStatus;
			
			ROS_INFO("Now we make them turn.");
			sendMoveOrder(2.0, 1.8);

			ros::Duration duration(0.25);
			int i;
			for (i=0 ; ros::ok() && i < 20 ; i++)
			{
				if (vectorDist(ourInitialStatus, m_ourStatus) > MARGIN_ERR && vectorDist(theirInitialStatus, m_theirStatus) > MARGIN_ERR)
					break;
				duration.sleep();
				ros::spinOnce();
			}
			checkRosOk(false);

			if (i >= 20)
			{
				ROS_WARN("At least one turtle is not moving.");
				return false;
			}
			
			ROS_INFO("Ok, turtles are both moving. Waiting for completion of one circle.");
			ros::Rate rate(10);
			while (ros::ok())
			{
				ros::spinOnce();
				if (vectorDist(ourInitialStatus, m_ourStatus) <= MARGIN_ERR || vectorDist(theirInitialStatus, m_theirStatus) <= MARGIN_ERR)
					break;
				sendMoveOrder(2.0, 1.8);
				rate.sleep();			
			}
			checkRosOk(false);
			
			ROS_INFO("At least one turtle reached its destination!");
			sendMoveOrder(0.0, 0.0);
			return true;
		}

		void blink()
		{
			ros::service::waitForService("clear");
			ros::ServiceClient clearService = m_nodeHandle.serviceClient<std_srvs::Empty>("clear");
			std_srvs::Empty empty;
			
			ROS_INFO("Blinking screen!");
			m_nodeHandle.setParam("/background_b", 0);
			clearService.call(empty);
			ros::Duration(0.5).sleep();
			m_nodeHandle.setParam("/background_b", 255);
			clearService.call(empty);
		}
};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "turtlemove");
	ROS_INFO("Initialized ROS.");

	ros::NodeHandle nodeHandle;
	MyLittleTurtles turtles(nodeHandle);
	if (turtles.circle())
		turtles.blink();

	ROS_INFO("Bye!");
	return 0;
}


