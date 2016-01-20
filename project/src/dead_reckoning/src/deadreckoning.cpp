#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <complex>
#include <SDL/SDL.h>

#include "../../utilities.h"

using namespace std;

struct Vector
{
    double x, y;
};

static SDL_Color createColor(int r, int g, int b)
{
    SDL_Color color = {r, g, b};
    return color;
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

struct BooleanPoint
{
    double x, y;
    ros::Time t;
    bool p;
};

class Grid
{
    private:
        static ros::Time timeSubstract(const ros::Time& t, const ros::Duration& d)
        {
            return t.toSec() > d.toSec() ? (t-d) : ros::Time(0);   
        }
        
        double m_precision;
        ros::Duration m_ttl;
        double m_minX, m_maxX, m_minY, m_maxY;
        std::list<BooleanPoint>** m_data;
        int m_height, m_width;

        void updateSize()
        {
            m_minX = m_precision * round(m_minX / m_precision);
            m_minY = m_precision * round(m_minY / m_precision);
            m_maxX = m_precision * round(m_maxX / m_precision);
            m_maxY = m_precision * round(m_maxY / m_precision);
            m_height = round((m_maxY - m_minY) / m_precision);
            m_width = round((m_maxX - m_minX) / m_precision);
        }
        
        bool _get(int ix, int iy)
        {
            if (ix >= m_width || iy >= m_height)
                return false;
            
            int k = ix*m_height+iy;
            if (m_data[k] == NULL)
                return false;
            ros::Time minTime = timeSubstract(ros::Time::now(), m_ttl);
            while (!m_data[k]->empty() && m_data[k]->front().t < minTime)
                m_data[k]->pop_front();
            //TODO filtering
            return (m_data[k] == NULL || m_data[k]->empty()) ? false : m_data[k]->back().p;
        }

        

    public:
        Grid(double precision=0.1, ros::Duration ttl=ros::Duration(120.0), double minX=0, double maxX=10, double minY=0, double maxY=10):
            m_precision(precision), m_ttl(ttl), m_minX(minX), m_maxX(maxX), m_minY(minY), m_maxY(maxY)
        {
            updateSize();
            m_data = new std::list<BooleanPoint>*[m_height*m_width];
            //TODO check if m_data != NULL
            memset(m_data, 0, sizeof(std::list<BooleanPoint>*)*m_height*m_width);
        }

        ~Grid()
        {
            if (m_data != NULL)
            {
                int n = m_width*m_height;
                for (int i=0 ; i < n ; i++)
                {
                    if (m_data[i] != NULL)
                        delete m_data[i];
                }
                delete m_data;
            }
        }
        
        double precision() const
        {
            return m_precision;
        }

        void addPoint(double x, double y, ros::Time t, bool p)
        {
            BooleanPoint point = {x, y, t, p};
            addPoint(point);
        }
        
        void addPoint(BooleanPoint point)
        {
            double x = m_precision * round(point.x / m_precision);
            double y = m_precision * round(point.y / m_precision);
            
            //ROS_INFO("Add point to grid at (%.3f, %.3f)", x, y);

            if (x < m_minX || x > m_maxX || y < m_minY || y > m_maxX)
            {
                return;

                //ROS_INFO("Resizing grid");

                int xShift = 0;
                if (x < m_minX)
                {
                    xShift = round((m_minX - x) / m_precision);
                    m_minX = x;
                }
                else if (x > m_maxX)
                    m_maxX = std::max(m_maxX+1, x);
                
                int yShift = 0;
                if (y < m_minY)
                {
                    yShift = round((m_minY - y) / m_precision);
                    m_minY = y;
                }
                else if (y > m_maxY)
                    m_maxY = std::max(m_maxY+1, y);
                
                int prevWidth = m_width;
                int prevHeight = m_height;
                updateSize();
                std::list<BooleanPoint>** newData = new std::list<BooleanPoint>*[m_height*m_width];
                memset(newData, 0, sizeof(std::list<BooleanPoint>*)*m_height*m_width);
                
                for (int i=0 ; i < prevWidth ; i++)
                    memcpy(&(newData[(i+xShift)*m_height+yShift]), &(m_data[prevHeight*i]), sizeof(std::list<BooleanPoint>*)*prevHeight);
                
                delete m_data;
                m_data = newData;
            }
            
            int ix = round((point.x-m_minX) / m_precision);
            int iy = round((point.y-m_minY) / m_precision);
            //ROS_INFO("Grid coord: (%d, %d)", ix, iy);
            
            int k = ix*m_height+iy;
            
            if (m_data[k] == NULL)
                m_data[k] = new std::list<BooleanPoint>;
            ros::Time minTime = timeSubstract(ros::Time::now(), m_ttl);
            int n = m_data[k]->size();
            while (n > 100 || (n > 0 && m_data[k]->front().t < minTime))
            {
                m_data[k]->pop_front();
                n--;
            }
            if (m_data[k]->empty() || m_data[k]->back().t < point.t)
                m_data[k]->push_back(point);

            //ROS_INFO("Added to grid");
        }
        
        bool get(double x, double y)
        {
            int ix = round((x-m_minX) / m_precision);
            int iy = round((y-m_minY) / m_precision);
            return _get(ix, iy);
        }
        
        SDL_Surface* draw(int w, int h, SDL_Color color, SDL_Surface *surf=NULL)
        {
            //ROS_INFO("Drawing grid");
            if (surf == NULL)
                surf = SDL_CreateRGBSurface(SDL_HWSURFACE, w, h, 32,0,0,0,0);
            if (surf == NULL)
                return NULL;
            
            //ROS_INFO("Surface created");
            Uint32 pixel = SDL_MapRGB(surf->format, color.r, color.g, color.b);
            SDL_LockSurface(surf);
            for (int y=0 ; y < h ; y++)
            {
                int ix = y * m_height / h;
                for (int x=0 ;  x < w ; x++)
                {
                    int iy = x * m_width / w;
                    //ROS_INFO("Drawing at (%d,%d) (%d, %d)", ix, iy, x, y);
                    if (_get(ix, iy))
                        putPixel(surf, x, h-y, pixel, true);
                }
            }
            SDL_UnlockSurface(surf);
            
            //ROS_INFO("Grid drawn");
            return surf;
        }
};

class MyLittleDisplay
{
    private:
        int m_width;
        int m_height;
        double m_maxX;
        double m_maxY;
        SDL_Surface *m_screen;
        SDL_Surface *m_robotSurf;
        bool m_ok;

        void blitRobot(int x, int y)
        {
            if (!m_robotSurf)
                return;
            
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

        void update(Vector pos, Vector speed, Vector acceleration, Vector *cloudPoints, size_t nbPoints, Vector targetPoint, Grid& grid)
        {
            SDL_Surface *gridSurf = SDL_CreateRGBSurface(SDL_HWSURFACE, m_width, m_height, 32,0,0,0,0);
            SDL_FillRect(gridSurf, NULL, SDL_MapRGB(gridSurf->format, 255,255,255));
            grid.draw(m_width, m_height, createColor(128,128,128), gridSurf);
            SDL_Rect rect = {0};
            SDL_BlitSurface(gridSurf, NULL, m_screen, &rect);
            
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
            drawLine(m_screen, pos.y*kx, m_height-pos.x*ky, targetPoint.y*kx, m_height-targetPoint.x*ky, createColor(255,128,0));

            SDL_Flip(m_screen);
        }
};

class RobotPilot
{
    private:
        static const double ANGLE_PRECISION;  //deg
        static const int NB_CLOUDPOINTS;
        
        ros::NodeHandle& m_node;
        ros::Publisher m_commandPub;    // Publisher to the robot's velocity command topic
        ros::Subscriber m_laserSub;     // Subscriber to the robot's laser scan topic
        double *m_ranges;
        bool m_simulation;
        ros::Time m_time;
        geometry_msgs::Vector3 m_position;
        double m_linearSpeed;
        double m_angularSpeed;
        geometry_msgs::Vector3 m_targetPoint;
        Vector *m_cloudPoints;
        int m_cloudPointsStartIdx;
        Grid m_grid;
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
                m_position.x += r * (sin(deltaAngle + m_position.z) - sin(m_position.z));
                m_position.y -= r * (cos(deltaAngle + m_position.z) - cos(m_position.z));
                m_position.z += deltaAngle;
            }
            else
            {
                m_position.x += m_linearSpeed * deltaTime * cos(m_position.z);
                m_position.y += m_linearSpeed * deltaTime * sin(m_position.z);
            }

            m_linearSpeed = lin;
            m_angularSpeed = ang;

            geometry_msgs::Twist msg;     // The default constructor will set all commands to 0
            msg.linear.x = lin;
            msg.angular.z = -ang;
            m_commandPub.publish(msg);

            m_time = ros::Time::now();
        }

        // Process the incoming laser scan message
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
        {
            int maxIdx = ceil(360 / ANGLE_PRECISION);
            int nbRanges = ceil((scan->angle_max - scan->angle_min) / scan->angle_increment);
            int prevAngleIdx = -1;
            ros::Time t = ros::Time::now();
            
            for (int i=0 ; i < nbRanges ; i++)
            {
                double angle = modAngle(-(scan->angle_min + i * scan->angle_increment));
                int angleIdx = round(angle * 180 / (M_PI * ANGLE_PRECISION));
                if (angleIdx >= maxIdx)
                    angleIdx = 0;
                if (angleIdx != prevAngleIdx)
                {
                    prevAngleIdx = angleIdx;
                    m_ranges[angleIdx] = scan->ranges[i];
                }
                else if (m_ranges[angleIdx] > scan->ranges[i])
                    m_ranges[angleIdx] = scan->ranges[i];
                
                double cosAngle = cos(angle + m_position.z);
                double sinAngle = sin(angle + m_position.z);
                double endX = scan->ranges[i] * cosAngle + m_position.x;
                double endY = scan->ranges[i] * sinAngle + m_position.y;
                m_grid.addPoint(endX, endY, t, true);
                
                double dstep = m_grid.precision();
                double dmax = scan->ranges[i] - dstep;
                for (double d=dstep ; d <= dmax ; d += dstep)
                {
                    double x = d * cosAngle + m_position.x;
                    double y = d * sinAngle + m_position.y;
                    m_grid.addPoint(x, y, t, false);
                }
                
                int cloudPointIdx = (i+m_cloudPointsStartIdx) % NB_CLOUDPOINTS;
                m_cloudPoints[cloudPointIdx].x = endX;
                m_cloudPoints[cloudPointIdx].y = endY;
            }
            m_cloudPointsStartIdx += nbRanges;
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

        void updateDisplay(geometry_msgs::Vector3 tgtPoint)
        {
            Vector pos = {m_position.x, m_position.y};
            Vector speed = {m_linearSpeed * cos(m_position.z), m_linearSpeed * sin(m_position.z)};
            Vector acceleration = {-m_linearSpeed * m_angularSpeed * sin(m_position.z), m_linearSpeed * m_angularSpeed * cos(m_position.z)};
            Vector targetPoint = {tgtPoint.x, tgtPoint.y};
            //ROS_INFO("Updating display");
            m_display.update(pos, speed, acceleration, m_cloudPoints, NB_CLOUDPOINTS, targetPoint, m_grid);
            //ROS_INFO("Display updated");
        }

    public:
        static const double MIN_PROXIMITY_RANGE;
        static const double MAX_LINEARSPEED;
        static const double MAX_ANGULARSPEED;

        RobotPilot(ros::NodeHandle& node, bool simulation): m_node(node), m_simulation(simulation),
            m_angularSpeed(0), m_linearSpeed(0), m_cloudPointsStartIdx(0), m_display(512, 512, 10.0, 10.0), m_grid(0.1)
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
            m_laserSub = m_node.subscribe("/scan", 1, &RobotPilot::scanCallback, this);
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

        ~RobotPilot()
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
            bool isStuck = false;
            
            ros::Time targetETA;
            geometry_msgs::Vector3 targetPoint;
            
            for (int i=0 ; ros::ok() ; i++)
            {
                ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
                
                if (i % mainModulo == 0)
                {
                    targetETA = ros::Time::now() + ros::Duration(deltaTime);
                    targetPoint.x = 10 * (rand() / (double)RAND_MAX);
                    targetPoint.y = 10 * (rand() / (double)RAND_MAX);
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
                
                updateDisplay(targetPoint);
                rate.sleep();
            }
        }

        bool goTo(const geometry_msgs::Vector3& targetPoint, double remainingTime)
        {
            static unsigned int lastProxAlert = 0;
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
                if (ros::Time::now().toSec() - lastProxAlert >= 1.0)
                {
                    lastProxAlert = ros::Time::now().toSec();
                    ROS_INFO("Proximity alert!");
                }
                linearSpeed = 0;
                ok = false;
            }

            sendMoveOrder(linearSpeed, angularSpeed);
            return ok;
        }
};

const double RobotPilot::ANGLE_PRECISION = 1.0; //deg
const int RobotPilot::NB_CLOUDPOINTS = 1000;
const double RobotPilot::MIN_PROXIMITY_RANGE = 0.5; // Should be smaller than sensor_msgs::LaserScan::range_max
const double RobotPilot::MAX_LINEARSPEED = 0.5;
const double RobotPilot::MAX_ANGULARSPEED = M_PI/4;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "deadreckoning");
    ros::NodeHandle node;
    ROS_INFO("Initialized ROS.");

    bool simulation = true;
    if (argc > 1 && strcmp(argv[1], "realworld"))
        simulation = false;
    ROS_INFO("Mode: %s", simulation ? "Simulation" : "Real world");

    srand(time(0));
    
    RobotPilot pilot(node, simulation);
    pilot.startMoving();
    
    ROS_INFO("Bye!");
    return 0;
};

