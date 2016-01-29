#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <complex>
#include <SDL/SDL.h>
#include <SDL/SDL_image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_broadcaster.h>
#include "dead_reckoning/Grid.h"
#include "detect_marker/MarkerInfo.h"
#include "detect_marker/MarkersInfos.h"
#include "sdl_gfx/SDL_rotozoom.h"

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
        
        double minX() const
        {
            return m_minX;
        }
        
        double minY() const
        {
            return m_minY;
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
                point.p = 1 - (1-point.p)*(1-m_data[k]->p);
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
        
        double* getAll(int* width=NULL, int *height=NULL, double *scale=NULL) const
        {
            double *data = new double[m_width*m_height];
            if (data == NULL)
            {
                ROS_ERROR("Unable to create double array (%dx%d)", m_width, m_height);
                return NULL;
            }
            
            for (int x=0 ; x < m_width ; x++)
            {
                int k = x * m_height;
                for (int y=0 ; y < m_height ; y++)
                {
                    ProbabilisticPoint *pt = m_data[k+y];
                    data[k+y] = pt != NULL ? pt->p : -1;
                }
            }
            if (width != NULL)
                *width = m_width;
            if (height != NULL)
                *height = m_height;
            if (scale != NULL)
                *scale = m_precision;
            return data;
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
        static const int SCREEN_HEIGHT;
        static const int SCREEN_WIDTH;
        static const std::string LOCALMAP_SCAN_TRANSFORM_NAME;
        static const std::string LOCALMAP_DEPTH_TRANSFORM_NAME;
        static const std::string ROBOTPOS_TRANSFORM_NAME;
        static const std::string SCANGRIDPOS_TRANSFORM_NAME;
        static const std::string DEPTHGRIDPOS_TRANSFORM_NAME;
        static const std::string MARKERPOS_TRANSFORM_NAME;
        static const double MARKER_SIZE;
        static const double MARKER_REF_DIST;
        
        static double modAngle(double rad)
        {
            return fmod(fmod(rad, 2*M_PI) + 2*M_PI, 2*M_PI);
        }
        
        ros::NodeHandle& m_node;
        ros::Subscriber m_orderSub;
        ros::Subscriber m_laserSub;
        ros::Subscriber m_depthSub;
        ros::Subscriber m_imuSub;
        ros::Subscriber m_localMapScanSub, m_localMapDepthSub;
        ros::Subscriber m_markersSub;
        ros::Publisher m_laserScanPub, m_laserDepthPub;
        ros::Publisher m_scanGridPub, m_depthGridPub;
        double *m_scanRanges, *m_depthRanges;
        bool m_simulation;
        ros::Time m_time;
        geometry_msgs::Vector3 m_position;
        double m_startOrientation;
        double m_linearSpeed;
        double m_angularSpeed;
        Vector *m_scanCloudPoints, *m_depthCloudPoints;
        int m_scanCloudPointsStartIdx, m_depthCloudPointsStartIdx;
        Grid m_scanGrid, m_depthGrid;
        SDL_Surface *m_screen;
        SDL_Surface *m_robotSurf, *m_markerSurf;
        SDL_Surface *m_gridSurf;
        double m_minX, m_maxX, m_minY, m_maxY;
        tf::TransformBroadcaster m_transformBroadcaster;
        Vector m_markersPos[256];
        
        void markersCallback(const detect_marker::MarkersInfos::ConstPtr& markersInfos)
        {
            for (std::vector<detect_marker::MarkerInfo>::const_iterator it = markersInfos->infos.begin() ; it != markersInfos->infos.end() ; it++)
            {
                if (it->id < 0 || it->id > 255)
                    continue;
                double angle = -atan(it->x*MARKER_SIZE / (2*MARKER_REF_DIST));
                double d = it->d / cos(angle);
                angle += m_position.z;
                m_markersPos[it->id].x = d * cos(angle) + m_position.x;
                m_markersPos[it->id].y = d * sin(angle) + m_position.y;
            }
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
        
        void localMapScanCallback(const nav_msgs::OccupancyGrid::ConstPtr& occ)
        {
            //ROS_INFO("Received local map");
            updateGridFromOccupancy(occ, m_scanGrid);
            publishGrid(m_scanGrid, m_scanGridPub);
        }
        
        void localMapDepthCallback(const nav_msgs::OccupancyGrid::ConstPtr& occ)
        {
            //ROS_INFO("Received local map");
            updateGridFromOccupancy(occ, m_depthGrid);
            publishGrid(m_depthGrid, m_depthGridPub);
        }
        
        void updateGridFromOccupancy(const nav_msgs::OccupancyGrid::ConstPtr& occ, Grid& grid)
        {
            ros::Time t = ros::Time::now();
            for (int y=0 ; y < occ->info.height ; y++)
            {
                int k = y * occ->info.width;
                double fy = m_position.y + (y-(int)occ->info.height/2)*occ->info.resolution;
                for (int x=0 ; x < occ->info.width ; x++)
                {
                    double p = occ->data[k+x] / 100.0;
                    if (p >= 0)
                    {
                        //ROS_INFO("Point at (%d, %d): %.3f", x, y, p);
                        double fx = m_position.x + (x-(int)occ->info.width/2)*occ->info.resolution;
                        grid.addPoint(fx, fy, t, p);
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

                if (*iter_y > 0.5 || *iter_y < -0.5)
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

        void processLaserScan(sensor_msgs::LaserScan& scan, bool invert, double *ranges, Vector *cloudPoints, int& startIdx)
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
                    ranges[angleIdx] = range;
                }
                else if (ranges[angleIdx] > range)
                    ranges[angleIdx] = range;
                
                if (!std::isinf(range))
                {
                    double endX = range * cos(angle + m_position.z) + m_position.x;
                    double endY = range * sin(angle + m_position.z) + m_position.y;
                    int cloudPointIdx = (i+startIdx) % NB_CLOUDPOINTS;
                    cloudPoints[cloudPointIdx].x = endX;
                    cloudPoints[cloudPointIdx].y = endY;
                    //ROS_INFO("Added cloud point (%.3f, %.3f)", endX, endY);
                }
            }
            startIdx += nbRanges;
        }

        // Process the incoming laser scan message
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
        {
            sensor_msgs::LaserScan scanCopy = *scan;
            processLaserScan(scanCopy, !m_simulation, m_scanRanges, m_scanCloudPoints, m_scanCloudPointsStartIdx);
            
            scanCopy.header.frame_id = LOCALMAP_SCAN_TRANSFORM_NAME;
            m_laserScanPub.publish(scanCopy);
        }
        
        void depthCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud)
        {
            sensor_msgs::LaserScan scan;
            pointCloudToLaserScan(cloud, scan);
            processLaserScan(scan, false, m_depthRanges, m_depthCloudPoints, m_depthCloudPointsStartIdx);
            
            scan.header.frame_id = LOCALMAP_DEPTH_TRANSFORM_NAME;
            m_laserDepthPub.publish(scan);
        }
        
        void publishTransforms()
        {
            tf::Transform transform;
            tf::Quaternion q;
            
            transform.setOrigin( tf::Vector3(m_position.x, m_position.y, 0.0) );
            q.setRPY(0, 0, m_simulation ? m_position.z : modAngle(m_position.z+M_PI));
            transform.setRotation(q);
            m_transformBroadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", LOCALMAP_SCAN_TRANSFORM_NAME));
            
            if (!m_simulation)
            {
                transform.setOrigin( tf::Vector3(m_position.x, m_position.y, 0.0) );
                q.setRPY(0, 0, m_position.z);
                transform.setRotation(q);
                m_transformBroadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", LOCALMAP_DEPTH_TRANSFORM_NAME));
            }
            
            transform.setOrigin( tf::Vector3(m_position.x, m_position.y, 0.0) );
            q.setRPY(0, 0, m_position.z);
            transform.setRotation(q);
            m_transformBroadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", ROBOTPOS_TRANSFORM_NAME));
            
            transform.setOrigin( tf::Vector3(m_scanGrid.minX(), m_scanGrid.minY(), 0.0) );
            q.setRPY(0, 0, 0);
            transform.setRotation(q);
            m_transformBroadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", SCANGRIDPOS_TRANSFORM_NAME));
            
            if (!m_simulation)
            {
                transform.setOrigin( tf::Vector3(m_depthGrid.minX(), m_depthGrid.minY(), 0.0) );
                q.setRPY(0, 0, 0);
                transform.setRotation(q);
                m_transformBroadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", DEPTHGRIDPOS_TRANSFORM_NAME));
            }
        }
        
        void publishMarkersTransforms()
        {
            tf::Transform transform;
            tf::Quaternion q;
            char transformName[100];
            
            q.setRPY(0, 0, 0);
            transform.setRotation(q);
                
            for (int i=0 ; i < 256 ; i++)
            {
                if (isnan(m_markersPos[i].x) || isnan(m_markersPos[i].y))
                    continue;
                snprintf(transformName, 100, "%s_%d", MARKERPOS_TRANSFORM_NAME.c_str(), i);
                transform.setOrigin( tf::Vector3(m_markersPos[i].x, m_markersPos[i].y, 0.0) );
                m_transformBroadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", transformName));
            }
        }
        
        void publishGrid(const Grid& grid, ros::Publisher& pub)
        {
            dead_reckoning::Grid gridMsg;
            int width, height;
            double scale;
            double *data = grid.getAll(&width, &height, &scale);
            if (data == NULL)
                return;
            gridMsg.data.assign(data, data+width*height);
            gridMsg.width = width;
            gridMsg.height = height;
            gridMsg.scale = scale;
            gridMsg.x = grid.minX();
            gridMsg.y = grid.minY();
            pub.publish(gridMsg);
            delete data;
        }

        bool initSDL()
        {
            if (SDL_Init(SDL_INIT_EVERYTHING) != 0)
            {
                ROS_ERROR("Unable to initialize the SDL.");
                return false;
            }
            
            int flags = IMG_INIT_PNG;
            if((IMG_Init(flags) & flags) != flags)
            {
                ROS_ERROR("Unable to initialize IMG: %s", IMG_GetError());
                return false;
            }

            m_screen = SDL_SetVideoMode(SCREEN_WIDTH, SCREEN_HEIGHT, 32, SDL_SWSURFACE);
            if (!m_screen)
            {
                ROS_ERROR("Unable to create display window.");
                return false;
            }
            SDL_Flip(m_screen);
            
            std::string packagePath = "~";
            if (!m_node.getParam("package_path", packagePath))
               ROS_WARN("The package path is not set, it will default to '~'.");

            m_robotSurf = IMG_Load((packagePath + "/turtlebot_small.png").c_str());
            if (!m_robotSurf)
            {
                ROS_ERROR("Unable to load the robot bitmap (%s).", (packagePath + "/turtlebot_small.png").c_str());
                return false;
            }
            
            m_markerSurf = IMG_Load((packagePath + "/target_small.png").c_str());
            if (!m_markerSurf)
            {
                ROS_ERROR("Unable to load the marker bitmap (%s).", (packagePath + "/target_small.png").c_str());
                return false;
            }
            
            m_gridSurf = SDL_CreateRGBSurface(SDL_HWSURFACE, SCREEN_WIDTH, SCREEN_HEIGHT, 32,0,0,0,0);
            if (!m_gridSurf)
            {
                ROS_ERROR("Unable to create the grid surface.");
                return false;
            }
            SDL_SetColorKey(m_gridSurf, SDL_SRCCOLORKEY, SDL_MapRGB(m_gridSurf->format, 0,0,0));
            
            return true;
        }
        
        void updateDisplay()
        {
            //ROS_INFO("Updating display");
            
            Vector pos = {m_position.x, m_position.y};
            Vector speed = {m_linearSpeed * cos(m_position.z), m_linearSpeed * sin(m_position.z)};
            Vector acceleration = {-m_linearSpeed * m_angularSpeed * sin(m_position.z), m_linearSpeed * m_angularSpeed * cos(m_position.z)};
            
            SDL_Rect rect = {0};
            SDL_FillRect(m_screen, 0, SDL_MapRGB(m_screen->format, 255,255,255));
            
            SDL_FillRect(m_gridSurf, NULL, SDL_MapRGB(m_gridSurf->format, 0,0,0));
            m_scanGrid.draw(SCREEN_WIDTH, SCREEN_HEIGHT, m_minX, m_maxX, m_minY, m_maxY, m_gridSurf);
            SDL_BlitSurface(m_gridSurf, NULL, m_screen, &rect);
            
            if (!m_simulation)
            {
                SDL_FillRect(m_gridSurf, NULL, SDL_MapRGB(m_gridSurf->format, 0,0,0));
                m_depthGrid.draw(SCREEN_WIDTH, SCREEN_HEIGHT, m_minX, m_maxX, m_minY, m_maxY, m_gridSurf);
                SDL_BlitSurface(m_gridSurf, NULL, m_screen, &rect);
            }
            
            double kx = SCREEN_WIDTH / (m_maxX - m_minX);
            double ky = SCREEN_HEIGHT / (m_maxY - m_minY);
            
            Uint32 green = SDL_MapRGB(m_screen->format, 0,128,0);
            Uint32 blue = SDL_MapRGB(m_screen->format, 0,0,255);
            for (int i=0 ; i < NB_CLOUDPOINTS ; i++)
            {
                if (!isnan(m_scanCloudPoints[i].x) && !isnan(m_scanCloudPoints[i].y))
                    putPixel(m_screen, (m_scanCloudPoints[i].x-m_minX) * kx, SCREEN_HEIGHT - (m_scanCloudPoints[i].y-m_minY) * ky, green);
                if (!m_simulation && !isnan(m_depthCloudPoints[i].x) && !isnan(m_depthCloudPoints[i].y))
                    putPixel(m_screen, (m_depthCloudPoints[i].x-m_minX) * kx, SCREEN_HEIGHT - (m_depthCloudPoints[i].y-m_minY) * ky, blue);
            }

            int x, y;
            for (int i=0 ; i < 256 ; i++)
            {
                if (isnan(m_markersPos[i].x) || isnan(m_markersPos[i].y))
                    continue;
                convertPosToDisplayCoord(m_markersPos[i].x, m_markersPos[i].y, x, y);
                rect.x = x-m_markerSurf->w/2;
                rect.y = y-m_markerSurf->h/2;
                SDL_BlitSurface(m_markerSurf, NULL, m_screen, &rect);
            }
            
            convertPosToDisplayCoord(m_position.x, m_position.y, x, y);
            
            SDL_Surface *robotSurf = rotozoomSurface(m_robotSurf, m_position.z*180/M_PI, 1.0, 1);
            rect.x = x-robotSurf->w/2;
            rect.y = y-robotSurf->h/2;
            SDL_BlitSurface(robotSurf, NULL, m_screen, &rect);
            SDL_FreeSurface(robotSurf);
            
            drawLine(m_screen, x, y, x + 3*speed.x*kx, y - 3*speed.y*ky, createColor(0,0,255));
            drawLine(m_screen, x, y, x + 3*acceleration.x*kx, y - 3*acceleration.y*ky, createColor(255,0,0));
            drawLine(m_screen, x, y, x+30*cos(m_position.z), y-30*sin(m_position.z), createColor(0,0,0));
            
            SDL_Flip(m_screen);
            
            //ROS_INFO("Display updated.");
        }
        
        void convertPosToDisplayCoord(double fx, double fy, int& x, int& y)
        {
            double kx = SCREEN_WIDTH / (m_maxX - m_minX);
            double ky = SCREEN_HEIGHT / (m_maxY - m_minY);
            x = (fx-m_minX) * kx;
            y = SCREEN_HEIGHT - (fy-m_minY) * ky;
        }

    public:
        DeadReckoning(ros::NodeHandle& node, bool simulation=true, double minX=-5, double maxX=5, double minY=-5, double maxY=5):
            m_node(node), m_simulation(simulation),
            m_scanCloudPointsStartIdx(0), m_depthCloudPointsStartIdx(0),
            m_angularSpeed(0), m_linearSpeed(0),
            m_minX(minX), m_maxX(maxX), m_minY(minY), m_maxY(maxY)
        {
            initSDL();
            
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
            
            for (int i=0 ; i < 256 ; i++)
            {
                m_markersPos[i].x = nan("");
                m_markersPos[i].y = nan("");
            }
            
            m_startOrientation = m_position.z;
            m_scanGrid = Grid(0.05, ros::Duration(120.0), m_minX, m_maxX, m_minY, m_maxY, false);
            m_depthGrid = m_scanGrid;
            
            int nbRanges = ceil(360 / ANGLE_PRECISION);
            m_scanRanges = new double[nbRanges];
            //TODO exception if m_scanRanges == NULL
            for (int i=0 ; i < nbRanges ; i++)
                m_scanRanges[i] = nan("");
            
            m_depthRanges = new double[nbRanges];
            //TODO exception if m_depthRanges == NULL
            memcpy(m_depthRanges, m_scanRanges, sizeof(double)*nbRanges);

            m_scanCloudPoints = new Vector[NB_CLOUDPOINTS];
            //TODO exception if m_scanCloudPoints == NULL
            for (int i=0 ; i < NB_CLOUDPOINTS ; i++)
            {
                m_scanCloudPoints[i].x = nan("");
                m_scanCloudPoints[i].y = nan("");
            }
            
            m_depthCloudPoints = new Vector[NB_CLOUDPOINTS];
            //TODO exception if m_depthCloudPoints == NULL
            memcpy(m_depthCloudPoints, m_scanCloudPoints, sizeof(Vector)*NB_CLOUDPOINTS);
            
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
            ROS_INFO("Waiting for commands publisher...");
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
            
            m_laserScanPub = m_node.advertise<sensor_msgs::LaserScan>("/local_map_scan/scan", 10);
            m_localMapScanSub = m_node.subscribe<nav_msgs::OccupancyGrid>("/local_map_scan/local_map", 10, &DeadReckoning::localMapScanCallback, this);
            ROS_INFO("Waiting for scan local map...");
            while (ros::ok() && (m_localMapScanSub.getNumPublishers() <= 0 || m_laserScanPub.getNumSubscribers() <= 0))
                rate.sleep();
            checkRosOk_v();
            
            m_laserDepthPub = m_node.advertise<sensor_msgs::LaserScan>("/local_map_depth/scan", 10);
            m_localMapDepthSub = m_node.subscribe<nav_msgs::OccupancyGrid>("/local_map_depth/local_map", 10, &DeadReckoning::localMapDepthCallback, this);
            if (!m_simulation)
            {
                ROS_INFO("Waiting for depth local map...");
                while (ros::ok() && (m_localMapDepthSub.getNumPublishers() <= 0 || m_laserDepthPub.getNumSubscribers() <= 0))
                    rate.sleep();
                checkRosOk_v();
            }
            
            m_markersSub = m_node.subscribe<detect_marker::MarkersInfos>("/markerinfo", 10, &DeadReckoning::markersCallback, this);
            ROS_INFO("Waiting for marker infos...");
            while (ros::ok() && m_markersSub.getNumPublishers() <= 0)
                rate.sleep();
            checkRosOk_v();
            
            ROS_INFO("Creating grids publishers...");
            m_scanGridPub = m_node.advertise<dead_reckoning::Grid>("/dead_reckoning/scan_grid", 10);
            m_depthGridPub = m_node.advertise<dead_reckoning::Grid>("/dead_reckoning/depth_grid", 10);
            
            m_time = ros::Time::now();
            ROS_INFO("Ok, let's go.");
        }

        ~DeadReckoning()
        {
            if (m_scanRanges != NULL)
                delete m_scanRanges;
            if (m_scanCloudPoints != NULL)
                delete m_scanCloudPoints;
            if (m_depthRanges != NULL)
                delete m_depthRanges;
            if (m_depthCloudPoints != NULL)
                delete m_depthCloudPoints;
            if (m_gridSurf != NULL)
                SDL_FreeSurface(m_gridSurf);
            if (m_markerSurf != NULL)
                SDL_FreeSurface(m_markerSurf);
            IMG_Quit();
            SDL_Quit();
        }

        void reckon()
        {
            ROS_INFO("Starting reckoning.");
            ros::Rate rate(10);
            while (ros::ok())
            {
                ros::spinOnce();
                publishMarkersTransforms();
                publishTransforms();
                updateDisplay();
                rate.sleep();
            }
        }
};

const double DeadReckoning::ANGLE_PRECISION = 0.1; //deg
const int DeadReckoning::NB_CLOUDPOINTS = 1000;
const double DeadReckoning::MIN_PROXIMITY_RANGE = 0.5; // Should be smaller than sensor_msgs::LaserScan::range_max
const double DeadReckoning::MAX_RANGE = 15.0;
const int DeadReckoning::SCREEN_HEIGHT = 600;
const int DeadReckoning::SCREEN_WIDTH = 600;
const std::string DeadReckoning::LOCALMAP_SCAN_TRANSFORM_NAME = "localmap_pos_scan";
const std::string DeadReckoning::LOCALMAP_DEPTH_TRANSFORM_NAME = "localmap_pos_depth";
const std::string DeadReckoning::ROBOTPOS_TRANSFORM_NAME = "deadreckoning_robotpos";
const std::string DeadReckoning::SCANGRIDPOS_TRANSFORM_NAME = "deadreckoning_scangridpos";
const std::string DeadReckoning::DEPTHGRIDPOS_TRANSFORM_NAME = "deadreckoning_depthgridpos";
const std::string DeadReckoning::MARKERPOS_TRANSFORM_NAME = "deadreckoning_markerpos";
const double DeadReckoning::MARKER_SIZE = 0.175;
const double DeadReckoning::MARKER_REF_DIST = 0.20;
        
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

