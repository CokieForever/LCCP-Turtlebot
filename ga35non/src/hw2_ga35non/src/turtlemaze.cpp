#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <complex>
#include "utilities.h"

using namespace std;

#include "fn.hpp"

class MyLittleTurtleMaze
{
	private:
		struct Vector
		{
			double x, y;
		};
		
		static const double ANGLE_PRECISION;  //deg
		static const double MAX_RANGE;
		static const double EPS_0;
		
		ros::NodeHandle& m_node;
		ros::Publisher m_commandPub;	// Publisher to the robot's velocity command topic
		ros::Subscriber m_laserSub;		// Subscriber to the robot's laser scan topic
		bool m_keepMoving;
		double *m_ranges;

		static double modAngle(double rad)
		{
			return fmod(fmod(rad, 2*M_PI) + 2*M_PI, 2*M_PI);
		}

		// Send a velocity command
		void sendMoveOrder(double lin, double ang)
		{
			geometry_msgs::Twist msg; 	// The default constructor will set all commands to 0
			msg.linear.x = lin;
			msg.angular.z = ang;
			m_commandPub.publish(msg);
		}

		// Process the incoming laser scan message
		void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
		{
			int nbRanges = ceil((scan->angle_max - scan->angle_min) / scan->angle_increment);
			int prevAngleIdx = -1;
			for (int i=0 ; i < nbRanges ; i++)
			{
				double angle = modAngle(scan->angle_min + i * scan->angle_increment);
				int angleIdx = floor(angle * 180 / (M_PI * ANGLE_PRECISION));
				if (angleIdx != prevAngleIdx)
				{
					prevAngleIdx = angleIdx;
					m_ranges[angleIdx] = scan->ranges[i];
				}
				else if (m_ranges[angleIdx] > scan->ranges[i])
					m_ranges[angleIdx] = scan->ranges[i];
			}
		}

		Vector getResultingForce()
		{
			int nbRanges = ceil(360 / ANGLE_PRECISION);
			double angleDelta = ANGLE_PRECISION * M_PI / 180;
			Vector field = {0,0};
			double d1 = 0;
			double a1 = 0;
			bool jump = true;
			for (int i=0 ; i < nbRanges ; i++)
			{
				if (isnan(m_ranges[i]))
				{
					jump = true;
					continue;
				}

				if (jump)
				{
					d1 = m_ranges[i];
					a1 = (i * ANGLE_PRECISION) * M_PI / 180;
					jump = false;
					continue;
				}

				Vector field0 = field;

				double d2 = m_ranges[i];
				double a2 = (i * ANGLE_PRECISION) * M_PI / 180;
				if (fabs(d2 - d1) < 1e-6)
				{
					double d = (d1+d2) / 2;
					double k = (2*LINEAR_CHARGE*angleDelta) / (4*M_PI*EPS_0*d) * sin(angleDelta/2);
					field.x += k * cos((a1+a2)/2);
					field.y -= k * sin((a1+a2)/2);
				}
				else
				{
					double a = (d2-d1) / angleDelta;
					int sgna = a > 0 ? +1 : -1;
					double c = d2 * (angleDelta/(d2-d1)) - a2;
					double k = LINEAR_CHARGE / (4*M_PI*EPS_0*a);
					double k1 = r8_ci(fabs(a2+c)) - r8_ci(fabs(a1+c));
					double k2 = r8_si(fabs(a2+c)) - r8_si(fabs(a1+c));
					field.x += k * (cos(c)*k1 + sgna*sin(c)*k2);
					field.y -= k * (sgna*cos(c)*k2 - sin(c)*k1);
				}

				//ROS_INFO("From a1=%.0f to a2=%.0f, dx=%.f & dy=%.f", a1*180/M_PI, a2*180/M_PI, field.x-field0.x, field.y-field0.y);
				
				d1 = d2;
				a1 = a2;
			}

			Vector force = {-field.x * ROBOT_CHARGE, -field.y * ROBOT_CHARGE};
			return force;
		}

		bool proximityAlert()
		{
			int idx = ceil(45 / ANGLE_PRECISION);
			for (int i=0 ; i < idx ; i++)
			{
				if (m_ranges[i] <= MIN_PROXIMITY_RANGE)
					return true;
			}
			idx = ceil(360 / ANGLE_PRECISION);
			for (int i=floor(315 / ANGLE_PRECISION) ; i < idx ; i++)
			{
				if (m_ranges[i] <= MIN_PROXIMITY_RANGE)
					return true;
			}
			return false;
		}

	public:
		static const double MIN_PROXIMITY_RANGE;
		static const double ROBOT_CHARGE;
		static const double LINEAR_CHARGE;
		static const double ROBOT_MASS;
		static const double MAX_RAND_FORCE;
		static const double MAX_LINEARSPEED;
		static const double MAX_ANGULARSPEED;

		MyLittleTurtleMaze(ros::NodeHandle& node): m_keepMoving(true), m_node(node)
		{
			int nbRanges = ceil(360 / ANGLE_PRECISION);
			m_ranges = new double[nbRanges];
			//TODO exception if m_ranges == NULL
			for (int i=0 ; i < nbRanges ; i++)
				m_ranges[i] = nan("");

			// Subscribe to the simulated robot's laser scan topic
			m_laserSub = m_node.subscribe("/scan", 1, &MyLittleTurtleMaze::scanCallback, this);
			ROS_INFO("Waiting for laser scan...");
			ros::Rate rate(10);
			while (ros::ok() && m_laserSub.getNumPublishers() <= 0)
				rate.sleep();
			checkRosOk_v();
			
			// Advertise a new publisher for the simulated robot's velocity command topic
			m_commandPub = m_node.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);
			ROS_INFO("Waiting for a robot to control...");
			while (ros::ok() && m_commandPub.getNumSubscribers() <= 0)
				rate.sleep();
			checkRosOk_v();
		}

		~MyLittleTurtleMaze()
		{
			if (m_ranges != NULL)
				delete m_ranges;
		}

		void startMoving()
		{
			ROS_INFO("Starting movement.");
			
			double deltaTime = 1.0;
			double loopRate = 20.0;
			ros::Rate rate(loopRate);
			int mainModulo = round(loopRate * deltaTime);
			int subModulo = round(loopRate * 2.0);

			double linear = 0.0;
			double angular = 0.0;
			
			for (unsigned int i=0 ; ros::ok() ; i++)
			{
				ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
				
				if (i % mainModulo == 0)
				{
					Vector randomForce = {0.0, 0.0};
					if (i % subModulo == 0)
					{
						randomForce.x = MAX_RAND_FORCE * (rand() / (double)RAND_MAX);
						//randomForce.y = MAX_RAND_FORCE * (rand() / (double)RAND_MAX - 0.5);
						ROS_INFO("Random force: x=%.3f, y=%.3f", randomForce.x, randomForce.y);
					}

					Vector electroForce = getResultingForce();
					ROS_INFO("Electrostatic force: x=%.3f, y=%.3f", electroForce.x, electroForce.y);
					Vector force = {electroForce.x + randomForce.x, electroForce.y + randomForce.y};

					Vector acceleration = {force.x / ROBOT_MASS, force.y / ROBOT_MASS};
					Vector speed = {linear + deltaTime*acceleration.x, deltaTime*acceleration.y};
					ROS_INFO("Speed: x=%.3f, y=%.3f", speed.x, speed.y);
				
					double moveWeight = min(1.0, sqrt(speed.x*speed.x + speed.y*speed.y) / MAX_LINEARSPEED);
					double turnWeight = atan2(-speed.y, speed.x) / M_PI;
					ROS_INFO("Weights: l=%.3f, a=%.3f", moveWeight, turnWeight);
				
					linear = MAX_LINEARSPEED * moveWeight * pow((1-fabs(turnWeight)), 0.1);
					angular = MAX_ANGULARSPEED * moveWeight * turnWeight;

					ROS_INFO("Move: l=%.3f, a=%.0f", linear, angular * 180 / M_PI);
				}
				
				if (proximityAlert())
				{
					ROS_INFO("Proximity alert!");
					linear = 0;
				}

				sendMoveOrder(linear, angular);

				rate.sleep();
			}
		}
};

const double MyLittleTurtleMaze::ANGLE_PRECISION = 1.0; //deg
const double MyLittleTurtleMaze::EPS_0 = 8.85419e-12;
const double MyLittleTurtleMaze::MAX_RANGE = 20;

const double MyLittleTurtleMaze::ROBOT_CHARGE = 1e-4;
const double MyLittleTurtleMaze::LINEAR_CHARGE = 1e-5;
const double MyLittleTurtleMaze::ROBOT_MASS = 0.1;
const double MyLittleTurtleMaze::MAX_RAND_FORCE = 10;
const double MyLittleTurtleMaze::MIN_PROXIMITY_RANGE = 0.5; // Should be smaller than sensor_msgs::LaserScan::range_max
const double MyLittleTurtleMaze::MAX_LINEARSPEED = 15;
const double MyLittleTurtleMaze::MAX_ANGULARSPEED = M_PI * 5;


int main(int argc, char **argv)
{
	ros::init(argc, argv, "turtlemaze");
	ros::NodeHandle node;
	ROS_INFO("Initialized ROS.");

	srand(time(0));
	
	MyLittleTurtleMaze turtleMaze(node);
	turtleMaze.startMoving();
	
	ROS_INFO("Bye!");
	return 0;
};

