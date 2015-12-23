#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <complex>
#include <SDL/SDL.h>

#include "utilities.h"

using namespace std;

#include "fn.hpp"

struct Vector
{
	double x, y;
};

class MyLittleDisplay
{
	private:
		static SDL_Color createColor(int r, int g, int b)
		{
			SDL_Color color = {r, g, b};
			return color;
		}

		//From here: http://stackoverflow.com/questions/11737988/how-to-draw-a-line-using-sdl-without-using-external-libraries
		static void drawLine(SDL_Surface *surf, float x1, float y1, float x2, float y2, SDL_Color color)
		{
			Uint32 pixel = SDL_MapRGB(surf->format, color.r, color.g, color.b);

			SDL_LockSurface(surf);

			//Bresenham's line algorithm
			const bool steep = (fabs(y2 - y1) > fabs(x2 - x1));
			if(steep)
			{
				swap(x1, y1);
				swap(x2, y2);
			}

			if(x1 > x2)
			{
				swap(x1, x2);
				swap(y1, y2);
			}

			const float dx = x2 - x1;
			const float dy = fabs(y2 - y1);

			float error = dx / 2.0f;
			const int ystep = (y1 < y2) ? 1 : -1;
			int y = (int)y1;

			const int maxX = (int)x2;

			for(int x=(int)x1; x<maxX; x++)
			{
				if(steep)
					putPixel(surf, y,x, pixel);
				else
					putPixel(surf, x,y, pixel);

				error -= dy;
				if(error < 0)
				{
					y += ystep;
					error += dx;
				}
			}

			SDL_UnlockSurface(surf);
		}

		//From SDL documentation
		static bool putPixel(SDL_Surface *surface, int x, int y, Uint32 pixel, bool check=true)
		{
			if (check)
			{
				if (x < 0 || y < 0 || x >= surface->w || y >= surface->h)
					return false;
			}
		
			int bpp = surface->format->BytesPerPixel;
			/* Here p is the address to the pixel we want to set */
			Uint8 *p = (Uint8 *)surface->pixels + y * surface->pitch + x * bpp;

			switch(bpp)
			{
				case 1:
					*p = pixel;
					break;

				case 2:
					*(Uint16 *)p = pixel;
					break;

				case 3:
					if(SDL_BYTEORDER == SDL_BIG_ENDIAN)
					{
						p[0] = (pixel >> 16) & 0xff;
						p[1] = (pixel >> 8) & 0xff;
						p[2] = pixel & 0xff;
					}
					else
					{
						p[0] = pixel & 0xff;
						p[1] = (pixel >> 8) & 0xff;
						p[2] = (pixel >> 16) & 0xff;
					}
					break;

				case 4:
					*(Uint32 *)p = pixel;
					break;
			}

			return true;
		}

		int m_width;
		int m_height;
		double m_maxX;
		double m_maxY;
		SDL_Surface *m_screen;
		SDL_Surface *m_robotSurf;
		bool m_ok;

		void blitRobot(int x, int y)
		{
			SDL_Rect rect;
			rect.x = x-m_robotSurf->w/2;
			rect.y = y-m_robotSurf->h/2;
			SDL_BlitSurface(m_robotSurf, NULL, m_screen, &rect);
		}

	public:
		MyLittleDisplay(int w, int h, double maxX=0, double maxY=0): m_width(w), m_height(h), m_maxX(maxX), m_maxY(maxY), m_ok(false)
		{
			m_width = max(100, m_width);
			m_height = max(100, m_height);
			if (m_maxX <= 0)
				m_maxX = m_height;
			if (m_maxY <= 0)
				m_maxY = m_width;
			
			if (SDL_Init(SDL_INIT_EVERYTHING) != 0)
			{
				ROS_ERROR("Unable to initialize the SDL.");
				return;
			}

			m_screen = SDL_SetVideoMode(m_width, m_height, 32, SDL_SWSURFACE);
			if (!m_screen)
			{
				ROS_ERROR("Unable to create display window.");
				return;
			}
			SDL_Flip(m_screen);

			m_robotSurf = SDL_CreateRGBSurface(SDL_HWSURFACE, 20, 20, 32, 0,0,0,0);
			if (!m_robotSurf)
			{
				ROS_ERROR("Unable to create the robot bitmap.");
				return;
			}
			SDL_FillRect(m_robotSurf, NULL, SDL_MapRGB(m_robotSurf->format, 0,0,0));

			m_ok = true;
		}

		~MyLittleDisplay()
		{
			if (m_robotSurf)
				SDL_FreeSurface(m_robotSurf);
			SDL_Quit();
		}

		bool isOK()
		{
			return m_ok;
		}

		void update(Vector pos, Vector speed, Vector acceleration, Vector *cloudPoints, size_t nbPoints, Vector electroForce, Vector randomForce, Vector targetPoint)
		{
			if (!m_robotSurf)
				return;
			
			SDL_FillRect(m_screen, NULL, SDL_MapRGB(m_screen->format, 255,255,255));

			double kx = m_width / m_maxY;
			double ky = m_height / m_maxX;
			
			Uint32 black = SDL_MapRGB(m_screen->format, 0,0,0);
			for (int i=0 ; i < nbPoints ; i++)
			{
				if (!isnan(cloudPoints[i].x) && !isnan(cloudPoints[i].y))
					putPixel(m_screen, cloudPoints[i].y * kx, m_height - cloudPoints[i].x * ky, black);
			}

			blitRobot(pos.y * kx, m_height - pos.x * ky);
			drawLine(m_screen, pos.y*kx, m_height-pos.x*ky, (pos.y+speed.y*5)*kx, m_height-(pos.x+speed.x*5)*ky, createColor(0,0,255));
			drawLine(m_screen, pos.y*kx, m_height-pos.x*ky, (pos.y+acceleration.y*5)*kx, m_height-(pos.x+acceleration.x*5)*ky, createColor(255,0,0));
			drawLine(m_screen, pos.y*kx, m_height-pos.x*ky, (pos.y+electroForce.y*2)*kx, m_height-(pos.x+electroForce.x*2)*ky, createColor(0,255,0));
			drawLine(m_screen, pos.y*kx, m_height-pos.x*ky, (pos.y+randomForce.y*2)*kx, m_height-(pos.x+randomForce.x*2)*ky, createColor(0,128,0));
			drawLine(m_screen, pos.y*kx, m_height-pos.x*ky, targetPoint.y*kx, m_height-targetPoint.x*ky, createColor(255,128,0));

			SDL_Flip(m_screen);
		}
};

class MyLittleTurtleMaze
{
	private:
		static const double ANGLE_PRECISION;  //deg
		static const double EPS_0;
		static const int NB_CLOUDPOINTS;
		
		ros::NodeHandle& m_node;
		ros::Publisher m_commandPub;	// Publisher to the robot's velocity command topic
		ros::Subscriber m_laserSub;		// Subscriber to the robot's laser scan topic
		double *m_ranges;
		bool m_simulation;
		ros::Time m_time;
		geometry_msgs::Vector3 m_position;
		double m_linearSpeed;
		double m_angularSpeed;
		geometry_msgs::Vector3 m_targetPoint;
		Vector *m_cloudPoints;
		int m_cloudPointsStartIdx;
		MyLittleDisplay m_display;
		
		static double modAngle(double rad)
		{
			return fmod(fmod(rad, 2*M_PI) + 2*M_PI, 2*M_PI);
		}

		// Send a velocity command
		void sendMoveOrder(double lin, double ang)
		{
			double deltaTime = (ros::Time::now() - m_time).toSec();
			
			if (fabs(m_angularSpeed) > 1e-5)
			{
				double r = m_linearSpeed / m_angularSpeed;
				double deltaAngle = m_angularSpeed * deltaTime;
				double deltaX = fabs(r) * (1 - cos(deltaAngle));
				double deltaY = r * sin(deltaAngle);
				m_position.x += deltaY*cos(m_position.z) - deltaX*sin(m_position.z);
				m_position.y += deltaY*sin(m_position.z) + deltaX*cos(m_position.z);
				m_position.z += deltaAngle;
			}
			else
			{
				m_position.x += m_linearSpeed * deltaTime * cos(m_position.z);
				m_position.y += m_linearSpeed * deltaTime * sin(m_position.z);
			}

			m_linearSpeed = lin;
			m_angularSpeed = ang;

			geometry_msgs::Twist msg; 	// The default constructor will set all commands to 0
			msg.linear.x = lin;
			msg.angular.z = -ang;
			m_commandPub.publish(msg);

			m_time = ros::Time::now();
		}

		// Process the incoming laser scan message
		void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
		{
			int nbRanges = ceil((scan->angle_max - scan->angle_min) / scan->angle_increment);
			int prevAngleIdx = -1;
			for (int i=0 ; i < nbRanges ; i++)
			{
				double angle = modAngle(-(scan->angle_min + i * scan->angle_increment));
				int angleIdx = floor(angle * 180 / (M_PI * ANGLE_PRECISION));
				if (angleIdx != prevAngleIdx)
				{
					prevAngleIdx = angleIdx;
					m_ranges[angleIdx] = scan->ranges[i];
				}
				else if (m_ranges[angleIdx] > scan->ranges[i])
					m_ranges[angleIdx] = scan->ranges[i];

				int cloudPointIdx = (i+m_cloudPointsStartIdx) % NB_CLOUDPOINTS;
				m_cloudPoints[cloudPointIdx].x = scan->ranges[i] * cos(angle + m_position.z) + m_position.x;
				m_cloudPoints[cloudPointIdx].y = scan->ranges[i] * sin(angle + m_position.z) + m_position.y;
			}
			m_cloudPointsStartIdx += nbRanges;
		}

		Vector getResultingForce(bool useCloudPoints=true)
		{
			if (useCloudPoints)
				return getResultingForce_cloudPoints();
			else
				return getResultingForce_ranges();
		}

		Vector getResultingForce_ranges()
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
					field.y += k * sin((a1+a2)/2);
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
					field.y += k * (sgna*cos(c)*k2 - sin(c)*k1);
				}

				d1 = d2;
				a1 = a2;
			}

			Vector force = {-field.x * ROBOT_CHARGE, -field.y * ROBOT_CHARGE};
			return force;
		}

		Vector getResultingForce_cloudPoints()
		{
			Vector force = {0, 0};
			double k = -ROBOT_CHARGE*CLOUDPOINT_CHARGE / (4*M_PI*EPS_0);
			for (int i=0 ; i < NB_CLOUDPOINTS ; i++)
			{
				if (isnan(m_cloudPoints[i].x) || isnan(m_cloudPoints[i].y))
					continue;
				double n = k / (pow(m_cloudPoints[i].x - m_position.x, 2) + pow(m_cloudPoints[i].y - m_position.y, 2));
				double a = atan2(m_cloudPoints[i].y - m_position.y, m_cloudPoints[i].x - m_position.x);
				force.x += n * cos(a);
				force.y += n * sin(a);
			}
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

		void updateDisplay(Vector electroForce, Vector randomForce, geometry_msgs::Vector3 targetPoint)
		{
			Vector pos = {m_position.x, m_position.y};
			Vector speed = {m_linearSpeed * cos(m_position.z), m_linearSpeed * sin(m_position.z)};
			Vector acceleration = {-m_linearSpeed * m_angularSpeed * sin(m_position.z), m_linearSpeed * m_angularSpeed * cos(m_position.z)};
			Vector point = {targetPoint.x, targetPoint.y};
			m_display.update(pos, speed, acceleration, m_cloudPoints, NB_CLOUDPOINTS, electroForce, randomForce, point);
		}

	public:
		static const double MIN_PROXIMITY_RANGE;
		static const double ROBOT_CHARGE;
		static const double LINEAR_CHARGE;
		static const double CLOUDPOINT_CHARGE;
		static const double ROBOT_MASS;
		static const double MAX_RAND_FORCE;
		static const double MAX_LINEARSPEED;
		static const double MAX_ANGULARSPEED;

		MyLittleTurtleMaze(ros::NodeHandle& node, bool simulation): m_node(node), m_simulation(simulation),
			m_angularSpeed(0), m_linearSpeed(0), m_cloudPointsStartIdx(0), m_display(512, 512, 10.0, 10.0)
		{
			m_position.x = 2.0;
			m_position.y = 2.0;
			m_position.z = M_PI / 2;
			
			int nbRanges = ceil(360 / ANGLE_PRECISION);
			m_ranges = new double[nbRanges];
			//TODO exception if m_ranges == NULL
			for (int i=0 ; i < nbRanges ; i++)
				m_ranges[i] = nan("");

			m_cloudPoints = new Vector[NB_CLOUDPOINTS];
			//TODO exception if m_cloudPoints == NULL
			for (int i=0 ; i < NB_CLOUDPOINTS ; i++)
			{
				m_cloudPoints[i].x = nan("");
				m_cloudPoints[i].y = nan("");
			}

			// Subscribe to the robot's laser scan topic
			m_laserSub = m_node.subscribe("/scan", 1, &MyLittleTurtleMaze::scanCallback, this);
			ROS_INFO("Waiting for laser scan...");
			ros::Rate rate(10);
			while (ros::ok() && m_laserSub.getNumPublishers() <= 0)
				rate.sleep();
			checkRosOk_v();
			
			// Advertise a new publisher for the robot's velocity command topic
			m_commandPub = m_node.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);
			ROS_INFO("Waiting for a robot to control...");
			while (ros::ok() && m_commandPub.getNumSubscribers() <= 0)
				rate.sleep();
			checkRosOk_v();

			m_time = ros::Time::now();
		}

		~MyLittleTurtleMaze()
		{
			if (m_ranges != NULL)
				delete m_ranges;
			if (m_cloudPoints != NULL)
				delete m_cloudPoints;
		}

		void startMoving()
		{
			ROS_INFO("Starting movement.");
			
			double deltaTime = 5.0;
			double loopRate = 20.0;
			ros::Rate rate(loopRate);
			int mainModulo = round(loopRate * deltaTime);
			int subModulo = round(loopRate * 5.0);
			bool isStuck = false;
			
			ros::Time targetETA;
			geometry_msgs::Vector3 targetPoint;
			Vector electroForce, randomForce;
			
			for (int i=0 ; ros::ok() ; i++)
			{
				ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
				
				if (i % mainModulo == 0)
				{
					randomForce.x = 0.0;
					randomForce.y = 0.0;
					if (i % subModulo == 0)
					{
						randomForce.x = MAX_RAND_FORCE * (rand() / (double)RAND_MAX - 0.5);
						randomForce.y = MAX_RAND_FORCE * (rand() / (double)RAND_MAX - 0.5);
						ROS_INFO("Random force: x=%.3f, y=%.3f", randomForce.x, randomForce.y);
					}

					electroForce = getResultingForce();
					ROS_INFO("Electrostatic force: x=%.3f, y=%.3f", electroForce.x, electroForce.y);
					Vector force = {electroForce.x + randomForce.x, electroForce.y + randomForce.y};

					Vector acceleration = {force.x / ROBOT_MASS, force.y / ROBOT_MASS};
					acceleration.x -= m_linearSpeed * m_angularSpeed * sin(m_position.z);
					acceleration.y += m_linearSpeed * m_angularSpeed * cos(m_position.z);
					
					Vector speed = {m_linearSpeed * cos(m_position.z), m_linearSpeed * sin(m_position.z)};

					geometry_msgs::Vector3 position = m_position;
					position.x += acceleration.x * deltaTime*deltaTime/2 + speed.x * deltaTime;
					position.y += acceleration.y * deltaTime*deltaTime/2 + speed.y * deltaTime;
					position.z += m_angularSpeed * deltaTime;
					ROS_INFO("Target position: x=%.3f, y=%.3f", position.x, position.y);

					targetETA = ros::Time::now() + ros::Duration(deltaTime);
					targetPoint = position;
				}

				if (!goTo(targetPoint, (targetETA - ros::Time::now()).toSec()))
				{
					if (!isStuck)
					{
						i = -1;
						isStuck = true;
					}
				}
				else
					isStuck = false;
				
				updateDisplay(electroForce, randomForce, targetPoint);
				rate.sleep();
			}
		}

		bool goTo(const geometry_msgs::Vector3& targetPoint, double remainingTime)
		{
			if (remainingTime <= 0)
				return false;

			double a = modAngle(atan2(targetPoint.y-m_position.y, targetPoint.x-m_position.x) - m_position.z);
			if (a > M_PI)
				a -= 2*M_PI;
			double r = sqrt(pow(targetPoint.x-m_position.x, 2) + pow(targetPoint.y-m_position.y, 2));
			
			double angularSpeed = max(min(4.0 * a / remainingTime, MAX_ANGULARSPEED), -MAX_ANGULARSPEED);
			double linearSpeed = max(min(0.5 * r / remainingTime, MAX_LINEARSPEED), -MAX_LINEARSPEED);
			
			bool ok=true;
			if (proximityAlert())
			{
				ROS_INFO("Proximity alert!");
				linearSpeed = 0;
				ok = false;
			}

			sendMoveOrder(linearSpeed, angularSpeed);
			return ok;
		}
};

const double MyLittleTurtleMaze::ANGLE_PRECISION = 1.0; //deg
const double MyLittleTurtleMaze::EPS_0 = 8.85419e-12;
const int MyLittleTurtleMaze::NB_CLOUDPOINTS = 1000;

const double MyLittleTurtleMaze::ROBOT_CHARGE = 5e-6;
const double MyLittleTurtleMaze::LINEAR_CHARGE = 1e-5;
const double MyLittleTurtleMaze::CLOUDPOINT_CHARGE = 1e-8;
const double MyLittleTurtleMaze::ROBOT_MASS = 1.0;
const double MyLittleTurtleMaze::MAX_RAND_FORCE = 1;
const double MyLittleTurtleMaze::MIN_PROXIMITY_RANGE = 0.5; // Should be smaller than sensor_msgs::LaserScan::range_max
const double MyLittleTurtleMaze::MAX_LINEARSPEED = 0.5;
const double MyLittleTurtleMaze::MAX_ANGULARSPEED = M_PI/4;




int main(int argc, char **argv)
{
	ros::init(argc, argv, "turtlemaze");
	ros::NodeHandle node;
	ROS_INFO("Initialized ROS.");

	bool simulation = true;
	if (argc > 1 && strcmp(argv[1], "realworld"))
		simulation = false;
	ROS_INFO("Mode: %s", simulation ? "Simulation" : "Real world");

	srand(time(0));
	
	MyLittleTurtleMaze turtleMaze(node, simulation);
	turtleMaze.startMoving();
	
	ROS_INFO("Bye!");
	return 0;
};

