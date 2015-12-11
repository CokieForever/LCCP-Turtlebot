#include "ros/ros.h"

#include <math.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <turtlesim/Pose.h>

#include "utilities.h"

class MyLittleTurtle
{
	private:
		static const double MARGIN_ERR = 1e-2;
		ros::NodeHandle& m_nodeHandle;
		ros::Publisher m_turtlesimPublisher;
		ros::Subscriber m_turtlesimSubscriber;
		geometry_msgs::Vector3 m_status;
		
		void updateTurtleStatus(const turtlesim::Pose::ConstPtr& message)
		{
			m_status.x = message->x;
			m_status.y = message->y;
			m_status.z = message->theta;
		}

		bool isTurtleMoving(bool strict = false)
		{
			geometry_msgs::Vector3 prevStatus = m_status;
			ros::spinOnce();
			
			if (fabs(m_status.z-prevStatus.z) >= MARGIN_ERR)
				return true;
		
			if (!strict)
			{
				double d = (m_status.x-prevStatus.x)*(m_status.x-prevStatus.x) + (m_status.y-prevStatus.y)*(m_status.y-prevStatus.y);
				return d >= MARGIN_ERR*MARGIN_ERR;
			}
			else
			{
				double absAngle = fabs(fmod(m_status.z, 2*M_PI));
				bool isVertical = fabs(absAngle - M_PI/2) <= 0.1;
				bool isHorizontal = absAngle <= MARGIN_ERR || fabs(absAngle - M_PI) <= 0.1;
				return (fabs(m_status.x-prevStatus.x) >= MARGIN_ERR || isVertical) && (fabs(m_status.y-prevStatus.y) >= MARGIN_ERR || isHorizontal);
			}
		}

		bool waitForMovementStart()
		{
			ros::Duration duration(0.25);
			int i=0;
			for (; ros::ok() && i < 20 && !isTurtleMoving() ; i++)
				duration.sleep();
			return ros::ok() && i < 20;
		}

		bool waitForMovementEnd(bool strict = false)
		{
			if (!waitForMovementStart())
				return false;
			ros::Duration duration(0.25);
			do
			{
				duration.sleep();
			} while (isTurtleMoving(strict));
			return ros::ok();
		}

		void sendMoveOrder(double linear, double angular)
		{
			geometry_msgs::Twist twist;
			memset(&twist, 0, sizeof(geometry_msgs::Twist));
			twist.linear.x = linear;
			twist.angular.z = angular;
			m_turtlesimPublisher.publish(twist);
		}
	
	public:
		MyLittleTurtle(ros::NodeHandle& nodeHandle): m_nodeHandle(nodeHandle)
		{
			ROS_INFO("Publishing topic...");
			m_turtlesimPublisher = m_nodeHandle.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
			
			ROS_INFO("Subscribing to turtlesim topic...");
			m_turtlesimSubscriber = m_nodeHandle.subscribe("/turtle1/pose", 10, &MyLittleTurtle::updateTurtleStatus, this);

			ROS_INFO("Waiting for turtlesim...");
			ros::Rate loopRate(10);
			while (ros::ok() && (m_turtlesimPublisher.getNumSubscribers() <= 0 || m_turtlesimSubscriber.getNumPublishers() <= 0))
				loopRate.sleep();
			checkRosOk_v();
			
			ROS_INFO("Retrieving initial status...");
			ros::spinOnce();
			ROS_INFO("Initial status: x=%.3f, y=%.3f, z=%.3f", m_status.x, m_status.y, m_status.z);
			
			ROS_INFO("Everything's ready.");
		}

		void move()
		{
			ROS_INFO("Starting movement.");

			ROS_INFO("Stage 1: Moving forward...");
			sendMoveOrder(1.0, 0.0);
			if (!waitForMovementEnd(true))
				ROS_WARN("The turtle does not move. Trying rotation.");
			
			ROS_INFO("Stage 2: Rotating...");
			sendMoveOrder(0.0, M_PI/4);
			if (!waitForMovementEnd())
			{
				checkRosOk_v();
				ROS_WARN("The turtle does not rotate.");
				return;
			}

			ROS_INFO("Stage 3: Moving to the wall...");
			sendMoveOrder(1.0, 0.0);
			if (!waitForMovementStart())
			{
				checkRosOk_v();
				ROS_WARN("The turtle does not move.");
				return;
			}

			ROS_INFO("In movement. Waiting for collision...");
			ros::Duration duration(0.25);
			do
			{
				sendMoveOrder(1.0, 0.0);
				duration.sleep();
			} while (isTurtleMoving(true));
			checkRosOk_v();
			
			sendMoveOrder(0.0, 0.0);
			ROS_INFO("Well, it seems that the Turtle does not move anymore, or at least not correctly.");
			ROS_INFO("Final status: x=%.3f, y=%.3f, z=%.3f", m_status.x, m_status.y, m_status.z);
		}
};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "turtlemove");
	ROS_INFO("Initialized ROS.");

	ros::NodeHandle nodeHandle;
	MyLittleTurtle turtle(nodeHandle);
	turtle.move();

	ROS_INFO("Bye!");
	return 0;
}

