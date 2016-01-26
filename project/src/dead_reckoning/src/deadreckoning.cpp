#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <complex>
#include <SDL/SDL.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_broadcaster.h>

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

struct ProbabilisticPoint
{
    double x, y;
    ros::Time t;
    double p;
};

class Grid
{
    private:
        double m_precision;
        ros::Duration m_ttl;
        double m_minX, m_maxX, m_minY, m_maxY;
        ProbabilisticPoint** m_data;
        int m_height, m_width;
        bool m_expendable;
        
        void init()
        {
            updateSize();
            m_data = new ProbabilisticPoint*[m_height*m_width];
            //TODO check if m_data != NULL
            memset(m_data, 0, sizeof(ProbabilisticPoint*)*m_height*m_width);
            
            //ROS_INFO("(%d x %d) = %d", m_width, m_height, m_height*m_width);
        }
        
        void empty()
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
                m_data = NULL;
            }
        }

        void updateSize()
        {
            m_minX = m_precision * round(m_minX / m_precision);
            m_minY = m_precision * round(m_minY / m_precision);
            m_maxX = m_precision * round(m_maxX / m_precision);
            m_maxY = m_precision * round(m_maxY / m_precision);
            m_height = round((m_maxY - m_minY) / m_precision)+1;
            m_width = round((m_maxX - m_minX) / m_precision)+1;
        }
        
        double _get(int ix, int iy)
        {
            if (ix < 0 || iy < 0 || ix >= m_width || iy >= m_height)
                return -1;
            
            int k = ix*m_height+iy;
            int minTime = ros::Time::now().toSec() - m_ttl.toSec();
            if (m_data[k] == NULL || m_data[k]->t.toSec() < minTime)
                return -1;
            
            return m_data[k]->p;
        }

    public:
        Grid(double precision=0.05, ros::Duration ttl=ros::Duration(120.0), double minX=-10, double maxX=10, double minY=-10, double maxY=10, bool expendable=true):
            m_precision(precision), m_ttl(ttl), m_minX(minX), m_maxX(maxX), m_minY(minY), m_maxY(maxY), m_expendable(expendable)
        {
            init();
        }
        
        Grid(const Grid& grid):
            m_precision(grid.m_precision), m_ttl(grid.m_ttl), m_minX(grid.m_minX), m_maxX(grid.m_maxX), m_minY(grid.m_minY), m_maxY(grid.m_maxY), m_expendable(grid.m_expendable)
        {
            init();            
        }
        
        Grid& operator=(const Grid& grid)
        {
            empty();
            m_precision = grid.m_precision;
            m_ttl = grid.m_ttl;
            m_minX = grid.m_minX;
            m_maxX = grid.m_maxX;
            m_minY = grid.m_minY;
            m_maxY = grid.m_maxY;
            m_expendable = grid.m_expendable;
            init();
            return *this;
        }
        
        ~Grid()
        {
            empty();
        }
        
        double precision() const
        {
            return m_precision;
        }

        bool addPoint(double x, double y, ros::Time t, double p)
        {
            ProbabilisticPoint point = {x, y, t, p};
            return addPoint(point);
        }
        
        bool addPoint(ProbabilisticPoint point)
        {
            double x = m_precision * round(point.x / m_precision);
            double y = m_precision * round(point.y / m_precision);
            
            //ROS_INFO("Add point to grid at (%.3f, %.3f)", x, y);

            if (x < m_minX || x > m_maxX || y < m_minY || y > m_maxX)
            {
                //ROS_INFO("Out of grid: (%.3f, %.3f)", x, y);
                if (!m_expendable)
                    return false;

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
                ProbabilisticPoint** newData = new ProbabilisticPoint*[m_height*m_width];
                memset(newData, 0, sizeof(ProbabilisticPoint*)*m_height*m_width);
                
                for (int i=0 ; i < prevWidth ; i++)
                    memcpy(&(newData[(i+xShift)*m_height+yShift]), &(m_data[prevHeight*i]), sizeof(ProbabilisticPoint*)*prevHeight);
                
                delete m_data;
                m_data = newData;
            }
            
            int ix = round((x-m_minX) / m_precision);
            int iy = round((y-m_minY) / m_precision);
            //ROS_INFO("Grid coord: (%d, %d)", ix, iy);
            //ROS_INFO("(%d x %d)", m_width, m_height);
            
            int k = ix*m_height+iy;
            //ROS_INFO("%d", k);
            //ROS_INFO("%lu", sizeof(m_data)/sizeof(std::list<BooleanPoint>*));
            
            if (m_data[k] == NULL)
                m_data[k] = new ProbabilisticPoint;
            else if (point.t == m_data[k]->t)
                point.p = 1 - (1-point.p)*(m_data[k]->p);
            *(m_data[k]) = point;

            //ROS_INFO("Added to grid");
            return true;
        }
        
        double get(double x, double y)
        {
            int ix = round((x-m_minX) / m_precision);
            int iy = round((y-m_minY) / m_precision);
            return _get(ix, iy);
        }
        
        SDL_Surface* draw(int w, int h, double minX, double maxX, double minY, double maxY, SDL_Surface *surf=NULL)
        {
            //ROS_INFO("Drawing grid");
            if (surf == NULL)
                surf = SDL_CreateRGBSurface(SDL_HWSURFACE, w, h, 32,0,0,0,0);
            if (surf == NULL)
                return NULL;
            
            //ROS_INFO("Surface created");
            SDL_LockSurface(surf);
            for (int y=0 ; y < h ; y++)
            {
                double fy = (h-y) * (maxY-minY) / h + minY;
                for (int x=0 ;  x < w ; x++)
                {
                    double fx = x * (maxX-minX) / w + minX;
                    //ROS_INFO("Drawing at (%d,%d) (%d, %d)", ix, iy, x, y);
                    double p = get(fx, fy);
                    if (p >= 0)
                        putPixel(surf, x, y, SDL_MapRGB(surf->format, 255, 255*(1-p), 255*(1-p)), false);
                    else
                        putPixel(surf, x, y, SDL_MapRGB(surf->format, 200, 200, 255), false);
                }
            }
            SDL_UnlockSurface(surf);
            
            //ROS_INFO("Grid drawn");
            return surf;
        }
};

class DeadReckoning
{
    private:
        static const double ANGLE_PRECISION;  //deg
        static const int NB_CLOUDPOINTS;
        static const double MAX_RANGE;
        static const double MIN_PROXIMITY_RANGE;
        static const double MAX_LINEARSPEED;
        static const double MAX_ANGULARSPEED;
        static const int SCREEN_HEIGHT = 600;
        static const int SCREEN_WIDTH = 600;
        
        ros::NodeHandle& m_node;
        ros::Subscriber m_orderSub;
        ros::Subscriber m_laserSub;
        ros::Publisher m_laserPub;
        ros::Subscriber m_depthSub;
        ros::Subscriber m_imuSub;
        ros::Subscriber m_localMapSub;
        double *m_ranges;
        bool m_simulation;
        ros::Time m_time;
        geometry_msgs::Vector3 m_position;
        double m_startOrientation;
        double m_linearSpeed;
        double m_angularSpeed;
        Vector *m_cloudPoints;
        int m_cloudPointsStartIdx;
        Grid m_grid;
        SDL_Surface *m_screen;
        SDL_Surface *m_robotSurf;
        SDL_Surface *m_gridSurf;
        double m_minX, m_maxX, m_minY, m_maxY;
        
        static double modAngle(double rad)
        {
            return fmod(fmod(rad, 2*M_PI) + 2*M_PI, 2*M_PI);
        }

        void IMUCallback(const sensor_msgs::Imu::ConstPtr& imu)
        {
            if (!m_simulation)
                m_position.z = modAngle(2*asin(imu->orientation.z));
        }
        
        void moveOrderCallback(const geometry_msgs::Twist::ConstPtr& order)
        {
            //ROS_INFO("Received order: v=%.3f, r=%.3f", order->linear.x, order->angular.z);
            
            double deltaTime = (ros::Time::now() - m_time).toSec();
            
            if (fabs(m_angularSpeed) > 1e-5)
            {
                double r = m_linearSpeed / m_angularSpeed;
                double deltaAngle = m_angularSpeed * deltaTime;
                m_position.x += r * (sin(deltaAngle + m_position.z) - sin(m_position.z));
                m_position.y -= r * (cos(deltaAngle + m_position.z) - cos(m_position.z));
                if (m_simulation)
                    m_position.z += deltaAngle;
            }
            else
            {
                m_position.x += m_linearSpeed * deltaTime * cos(m_position.z);
                m_position.y += m_linearSpeed * deltaTime * sin(m_position.z);
            }

            m_linearSpeed = order->linear.x;
            m_angularSpeed = order->angular.z;
            m_time = ros::Time::now();
        }
        
        void localMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& grid)
        {
            //ROS_INFO("Received local map");

            ros::Time t = ros::Time::now();
            for (int y=0 ; y < grid->info.height ; y++)
            {
                int k = y * grid->info.width;
                double fy = /*(m_simulation ? 1 : -1) * */(m_position.y + (y-(int)grid->info.height/2)*grid->info.resolution);
                for (int x=0 ; x < grid->info.width ; x++)
                {
                    double p = grid->data[k+x] / 100.0;
                    if (p >= 0)
                    {
                        //ROS_INFO("Point at (%d, %d): %.3f", x, y, p);
                        double fx = /*(m_simulation ? 1 : -1) * */(m_position.x + (x-(int)grid->info.width/2)*grid->info.resolution);
                        m_grid.addPoint(fx, fy, t, p);
                    }
                }
            }
        }
        
        //From https://github.com/ros-perception/pointcloud_to_laserscan/blob/indigo-devel/src/pointcloud_to_laserscan_nodelet.cpp
        void pointCloudToLaserScan(const sensor_msgs::PointCloud2ConstPtr &cloud_msg, sensor_msgs::LaserScan& output)
        {
            output.angle_min = -30.0 * M_PI/180;
            output.angle_max = 30.0 * M_PI/180;
            output.angle_increment = ANGLE_PRECISION * M_PI / 180;
            output.time_increment = 0.0;
            output.scan_time = 1.0 / 30.0;
            output.range_min = 0.45;
            output.range_max = 15.0;

            //determine amount of rays to create
            uint32_t ranges_size = std::ceil((output.angle_max - output.angle_min) / output.angle_increment);
            output.ranges.assign(ranges_size, std::numeric_limits<double>::infinity());

            // Iterate through pointcloud
            for (sensor_msgs::PointCloud2ConstIterator<float>
                    iter_x(*cloud_msg, "x"), iter_y(*cloud_msg, "y"), iter_z(*cloud_msg, "z");
                    iter_x != iter_x.end();
                    ++iter_x, ++iter_y, ++iter_z)
            {

                if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z))
                    continue;

                if (*iter_y > 1.0 || *iter_y < -1.0)
                    continue;
                
                double range = hypot(*iter_x, *iter_z);
                if (range < output.range_min || range > output.range_max)
                    continue;

                //ROS_INFO("Range at (%.3f, %.3f): %.3f", *iter_x, *iter_y, range);
                
                double angle = -atan2(*iter_x, *iter_z);
                if (angle < output.angle_min || angle > output.angle_max)
                    continue;
                
                //ROS_INFO("Range at angle %.3f: %.3f", angle*180/M_PI, range);

                //overwrite range at laserscan ray if new range is smaller
                int index = (angle - output.angle_min) / output.angle_increment;
                if (range < output.ranges[index])
                {
                    output.ranges[index] = range;
                }
            }
        }

        void processLaserScan(sensor_msgs::LaserScan& scan, bool invert)
        {
            int maxIdx = ceil(360 / ANGLE_PRECISION);
            int nbRanges = ceil((scan.angle_max - scan.angle_min) / scan.angle_increment);
            int prevAngleIdx = -1;
            ros::Time t = ros::Time::now();
            
            for (int i=0 ; i < nbRanges ; i++)
            {
                if (scan.ranges[i] > MAX_RANGE || scan.ranges[i] >= scan.range_max)
                    scan.ranges[i] = std::numeric_limits<float>::infinity();
                double range = scan.ranges[i];
                
                double angle = modAngle(scan.angle_min + i * scan.angle_increment);
                if (invert)
                    angle = modAngle(angle + M_PI); //The laser scan points towards robot's back
                
                int angleIdx = round(angle * 180 / (M_PI * ANGLE_PRECISION));
                if (angleIdx >= maxIdx)
                    angleIdx = 0;
                if (angleIdx != prevAngleIdx)
                {
                    prevAngleIdx = angleIdx;
                    m_ranges[angleIdx] = range;
                }
                else if (m_ranges[angleIdx] > range)
                    m_ranges[angleIdx] = range;
                
                if (!std::isinf(range))
                {
                    double endX = range * cos(angle + m_position.z) + m_position.x;
                    double endY = range * sin(angle + m_position.z) + m_position.y;
                    int cloudPointIdx = (i+m_cloudPointsStartIdx) % NB_CLOUDPOINTS;
                    m_cloudPoints[cloudPointIdx].x = endX;
                    m_cloudPoints[cloudPointIdx].y = endY;
                    //ROS_INFO("Added cloud point (%.3f, %.3f)", endX, endY);
                }
            }
            m_cloudPointsStartIdx += nbRanges;
        }

        // Process the incoming laser scan message
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
        {
            sensor_msgs::LaserScan scanCopy = *scan;
            processLaserScan(scanCopy, !m_simulation);
            //publishLaserScan(scanCopy, !m_simulation);
        }
        
        void depthCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud)
        {
            sensor_msgs::LaserScan scan;
            pointCloudToLaserScan(cloud, scan);
            //processLaserScan(scan, false);
            publishLaserScan(scan, false);
        }

        void publishLaserScan(sensor_msgs::LaserScan& scan, bool invert)
        {
            static tf::TransformBroadcaster br;
            tf::Transform transform;
            transform.setOrigin( tf::Vector3(m_position.x, m_position.y, 0.0) );
            tf::Quaternion q;
            q.setRPY(0, 0, invert ? modAngle(m_position.z+M_PI) : m_position.z);
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "dr_pos"));
            
            scan.header.frame_id = "dr_pos";
            m_laserPub.publish(scan);
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

        bool initSDL()
        {
            if (SDL_Init(SDL_INIT_EVERYTHING) != 0)
            {
                ROS_ERROR("Unable to initialize the SDL.");
                return false;
            }

            m_screen = SDL_SetVideoMode(SCREEN_WIDTH, SCREEN_HEIGHT, 32, SDL_SWSURFACE);
            if (!m_screen)
            {
                ROS_ERROR("Unable to create display window.");
                return false;
            }
            SDL_Flip(m_screen);

            m_robotSurf = SDL_CreateRGBSurface(SDL_HWSURFACE, 20, 20, 32, 0,0,0,0);
            if (!m_robotSurf)
            {
                ROS_ERROR("Unable to create the robot bitmap.");
                return false;
            }
            
            SDL_FillRect(m_robotSurf, NULL, SDL_MapRGB(m_robotSurf->format, 0,0,0));
            return true;
        }
        
        void updateDisplay()
        {
            //ROS_INFO("Updating display");
            
            Vector pos = {m_position.x, m_position.y};
            Vector speed = {m_linearSpeed * cos(m_position.z), m_linearSpeed * sin(m_position.z)};
            Vector acceleration = {-m_linearSpeed * m_angularSpeed * sin(m_position.z), m_linearSpeed * m_angularSpeed * cos(m_position.z)};
            
            SDL_FillRect(m_screen, 0, SDL_MapRGB(m_screen->format, 255,255,255));
            
            SDL_Rect rect = {0};
            SDL_FillRect(m_gridSurf, NULL, SDL_MapRGB(m_gridSurf->format, 0,0,0));
            m_grid.draw(SCREEN_WIDTH, SCREEN_HEIGHT, m_minX, m_maxX, m_minY, m_maxY, m_gridSurf);
            SDL_BlitSurface(m_gridSurf, NULL, m_screen, &rect);
            
            double kx = SCREEN_WIDTH / (m_maxX - m_minX);
            double ky = SCREEN_HEIGHT / (m_maxY - m_minY);
            
            Uint32 black = SDL_MapRGB(m_screen->format, 0,0,0);
            for (int i=0 ; i < NB_CLOUDPOINTS ; i++)
            {
                if (!isnan(m_cloudPoints[i].x) && !isnan(m_cloudPoints[i].y))
                    putPixel(m_screen, (m_cloudPoints[i].x-m_minX) * kx, SCREEN_HEIGHT - (m_cloudPoints[i].y-m_minY) * ky, black);
            }

            int x = (m_position.x-m_minX) * kx;
            int y = SCREEN_HEIGHT - (m_position.y-m_minY) * ky;
            rect.x = x-m_robotSurf->w/2;
            rect.y = y-m_robotSurf->h/2;
            SDL_BlitSurface(m_robotSurf, NULL, m_screen, &rect);
            drawLine(m_screen, x, y, x + speed.x*kx, y - speed.y*ky, createColor(0,0,255));
            drawLine(m_screen, x, y, x + acceleration.x*kx, y - acceleration.y*ky, createColor(255,0,0));
            drawLine(m_screen, x, y, x+30*cos(m_position.z), y-30*sin(m_position.z), createColor(0,0,0));
            
            SDL_Flip(m_screen);
            
            //ROS_INFO("Display updated.");
        }

    public:
        DeadReckoning(ros::NodeHandle& node, bool simulation=true, double minX=-5, double maxX=5, double minY=-5, double maxY=5):
            m_node(node), m_simulation(simulation), m_cloudPointsStartIdx(0),
            m_angularSpeed(0), m_linearSpeed(0),
            m_minX(minX), m_maxX(maxX), m_minY(minY), m_maxY(maxY)
        {
            initSDL();
            
            m_gridSurf = SDL_CreateRGBSurface(SDL_HWSURFACE, SCREEN_WIDTH, SCREEN_HEIGHT, 32,0,0,0,0);
            SDL_SetColorKey(m_gridSurf, SDL_SRCCOLORKEY, SDL_MapRGB(m_gridSurf->format, 0,0,0));
            
            if (m_simulation)
            {
                m_position.x = 2.0;
                m_position.y = 2.0;
                m_position.z = 0.0;
            }
            else
            {
                m_position.x = 0.0;
                m_position.y = 0.0;
                m_position.z = M_PI/2;
            }
            
            m_startOrientation = m_position.z;
            m_grid = Grid(0.05, ros::Duration(120.0), m_minX, m_maxX, m_minY, m_maxY, false);
            
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
            m_laserSub = m_node.subscribe<sensor_msgs::LaserScan>("/scan", 1, &DeadReckoning::scanCallback, this);
            ROS_INFO("Waiting for laser scan...");
            ros::Rate rate(10);
            while (ros::ok() && m_laserSub.getNumPublishers() <= 0)
                rate.sleep();
            checkRosOk_v();
            
            // Subscribe to the robot's depth cloud topic
            m_depthSub = m_node.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 1, &DeadReckoning::depthCallback, this);
            if (!m_simulation)
            {
                ROS_INFO("Waiting for depth cloud...");
                while (ros::ok() && m_depthSub.getNumPublishers() <= 0)
                    rate.sleep();
                checkRosOk_v();
            }
            
            m_orderSub = m_node.subscribe<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000, &DeadReckoning::moveOrderCallback, this);
            ROS_INFO("Waiting for orders...");
            while (ros::ok() && m_orderSub.getNumPublishers() <= 0)
                rate.sleep();
            checkRosOk_v();
            
            m_imuSub = m_node.subscribe<sensor_msgs::Imu>("/mobile_base/sensors/imu_data", 1000, &DeadReckoning::IMUCallback, this);
            if (!m_simulation)
            {
                ROS_INFO("Waiting for IMU...");
                while (ros::ok() && m_imuSub.getNumPublishers() <= 0)
                    rate.sleep();
                checkRosOk_v();
            }
            
            m_laserPub = m_node.advertise<sensor_msgs::LaserScan>("/local_map/scan", 10);
            m_localMapSub = m_node.subscribe<nav_msgs::OccupancyGrid>("/local_map/local_map", 10, &DeadReckoning::localMapCallback, this);
            ROS_INFO("Waiting for local_map...");
            while (ros::ok() && (m_localMapSub.getNumPublishers() <= 0 || m_laserPub.getNumSubscribers() <= 0))
                rate.sleep();
            checkRosOk_v();
            
            m_time = ros::Time::now();
            ROS_INFO("Ok, let's go.");
        }

        ~DeadReckoning()
        {
            if (m_ranges != NULL)
                delete m_ranges;
            if (m_cloudPoints != NULL)
                delete m_cloudPoints;
            if (m_gridSurf != NULL)
                SDL_FreeSurface(m_gridSurf);
        }

        void reckon()
        {
            ROS_INFO("Starting reckoning.");
            ros::Rate rate(10);
            while (ros::ok())
            {
                ros::spinOnce();
                updateDisplay();
                rate.sleep();
            }
        }
};

const double DeadReckoning::ANGLE_PRECISION = 0.1; //deg
const int DeadReckoning::NB_CLOUDPOINTS = 1000;
const double DeadReckoning::MIN_PROXIMITY_RANGE = 0.5; // Should be smaller than sensor_msgs::LaserScan::range_max
const double DeadReckoning::MAX_LINEARSPEED = 0.5;
const double DeadReckoning::MAX_ANGULARSPEED = M_PI/4;
const double DeadReckoning::MAX_RANGE = 15.0;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "deadreckoning");
    ros::NodeHandle node("~");
    ROS_INFO("Initialized ROS.");

    bool simulation = true;
    std::string mode;
    if (node.getParam("mode", mode))
        simulation = mode != "realworld";
    ROS_INFO("Mode: %s", simulation ? "Simulation" : "Real world");

    srand(time(0));
    
    double minX=-5, maxX=5, minY=-5, maxY=5;
    if (simulation)
    {
        minX = 0; maxX = 10;
        minY = 0; maxY = 10;
    }
    DeadReckoning dr(node, simulation, minX, maxX, minY, maxY);
    dr.reckon();
    
    ROS_INFO("Bye!");
    return 0;
};

