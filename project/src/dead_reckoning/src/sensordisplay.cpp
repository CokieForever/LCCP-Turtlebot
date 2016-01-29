#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <SDL/SDL.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include "../../utilities.h"

struct Vector
{
    double x, y;
};

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

static double modAngle(double rad)
{
    return fmod(fmod(rad, 2*M_PI) + 2*M_PI, 2*M_PI);
}


class SensorDisplay
{
    private:
        static const int NB_SCANCLOUDPOINTS = 1000;
        static const int NB_DEPTHCLOUDPOINTS = 20000;
        static const int SCREEN_WIDTH = 600;
        static const int SCREEN_HEIGHT = 600;
        
        ros::NodeHandle& m_node;
        ros::Publisher m_commandPub;    // Publisher to the robot's velocity command topic
        ros::Subscriber m_laserSub;     // Subscriber to the robot's laser scan topic
        ros::Subscriber m_depthSub;
        bool m_simulation;
        Vector *m_scanCloudPoints;
        int m_scanCloudPointsStartIdx;
        Vector *m_depthCloudPoints;
        int m_depthCloudPointsStartIdx;
        SDL_Surface *m_screen;
        SDL_Surface *m_robotSurf;
        double m_minX, m_minY, m_maxX, m_maxY;
        
        // Process the incoming laser scan message
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
        {
            int nbRanges = ceil((scan->angle_max - scan->angle_min) / scan->angle_increment);
            for (int i=0 ; i < nbRanges ; i++)
            {
                double range = scan->ranges[i];
                double angle = modAngle(scan->angle_min + i * scan->angle_increment);
                double endX = range * cos(angle);
                double endY = range * sin(angle);
                
                int cloudPointIdx = (i+m_scanCloudPointsStartIdx) % NB_SCANCLOUDPOINTS;
                m_scanCloudPoints[cloudPointIdx].x = endY;
                m_scanCloudPoints[cloudPointIdx].y = -endX;
                //ROS_INFO("Added cloud point (%.3f, %.3f)", endX, endY);
            }
            m_scanCloudPointsStartIdx += nbRanges;
        }
        
        void depthCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud)
        {
            for (sensor_msgs::PointCloud2ConstIterator<float>
                    iter_x(*cloud, "x"), iter_y(*cloud, "y"), iter_z(*cloud, "z");
                    iter_x != iter_x.end();
                    ++iter_x, ++iter_y, ++iter_z)
            {

                if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z))
                    continue;
                
                if (*iter_y > 1.0 || *iter_y < -1.0)
                    continue;
                
                int cloudPointIdx = m_depthCloudPointsStartIdx % NB_DEPTHCLOUDPOINTS;
                m_depthCloudPoints[cloudPointIdx].x = *iter_x;
                m_depthCloudPoints[cloudPointIdx].y = *iter_z;
                
                m_depthCloudPointsStartIdx++;
            }
        }
        
        void updateDisplay()
        {
            SDL_Rect pos = {(SCREEN_WIDTH-m_robotSurf->w)/2, (SCREEN_HEIGHT-m_robotSurf->h)/2};
            SDL_FillRect(m_screen, NULL, SDL_MapRGB(m_screen->format, 255,255,255));
            SDL_BlitSurface(m_robotSurf, NULL, m_screen, &pos);
            
            double kx = SCREEN_WIDTH / (m_maxX-m_minX);
            double ky = SCREEN_HEIGHT / (m_maxY-m_minY);
            
            SDL_LockSurface(m_screen);
            
            Uint32 red = SDL_MapRGB(m_screen->format, 255,0,0);
            for (int i=0 ; i < NB_SCANCLOUDPOINTS ; i++)
            {
                if (!isnan(m_scanCloudPoints[i].x) && !isnan(m_scanCloudPoints[i].y))
                    putPixel(m_screen, (m_scanCloudPoints[i].x-m_minX) * kx, SCREEN_HEIGHT - (m_scanCloudPoints[i].y-m_minY) * ky, red);
            }
            
            Uint32 blue = SDL_MapRGB(m_screen->format, 0,0,255);
            for (int i=0 ; i < NB_DEPTHCLOUDPOINTS ; i++)
            {
                if (!isnan(m_depthCloudPoints[i].x) && !isnan(m_depthCloudPoints[i].y))
                    putPixel(m_screen, (m_depthCloudPoints[i].x-m_minX) * kx, SCREEN_HEIGHT - (m_depthCloudPoints[i].y-m_minY) * ky, blue);
            }
            
            SDL_UnlockSurface(m_screen);
            SDL_Flip(m_screen);
        }

    public:
        SensorDisplay(ros::NodeHandle& node, double maxX=5, double maxY=5, double minX=-5, double minY=-5):
            m_node(node), m_minX(minX), m_minY(minY), m_maxX(maxX), m_maxY(maxY)
        {
            m_scanCloudPoints = new Vector[NB_SCANCLOUDPOINTS];
            //TODO exception if m_cloudPoints == NULL
            for (int i=0 ; i < NB_SCANCLOUDPOINTS ; i++)
            {
                m_scanCloudPoints[i].x = nan("");
                m_scanCloudPoints[i].y = nan("");
            }
            
            m_depthCloudPoints = new Vector[NB_DEPTHCLOUDPOINTS];
            //TODO exception if m_cloudPoints == NULL
            for (int i=0 ; i < NB_DEPTHCLOUDPOINTS ; i++)
            {
                m_depthCloudPoints[i].x = nan("");
                m_depthCloudPoints[i].y = nan("");
            }
            
            SDL_Init(SDL_INIT_EVERYTHING);
            m_screen = SDL_SetVideoMode(SCREEN_WIDTH, SCREEN_HEIGHT, 32, SDL_SWSURFACE);
            SDL_Flip(m_screen);
            
            m_robotSurf = SDL_CreateRGBSurface(SDL_HWSURFACE, 10, 10, 32,0,0,0,0);
            
            // Subscribe to the robot's laser scan topic
            m_laserSub = m_node.subscribe<sensor_msgs::LaserScan>("/scan", 1, &SensorDisplay::scanCallback, this);
            ROS_INFO("Waiting for laser scan...");
            ros::Rate rate(10);
            while (ros::ok() && m_laserSub.getNumPublishers() <= 0)
                rate.sleep();
            checkRosOk_v();
            
            // Subscribe to the robot's depth cloud topic
            m_depthSub = m_node.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 1, &SensorDisplay::depthCallback, this);
            ROS_INFO("Waiting for depth cloud...");
            while (ros::ok() && m_depthSub.getNumPublishers() <= 0)
                rate.sleep();
            checkRosOk_v();
            
            ROS_INFO("Ok, everything's ready.");
        }
        
        ~SensorDisplay()
        {
            if (m_scanCloudPoints != NULL)
                delete m_scanCloudPoints;
            if (m_depthCloudPoints != NULL)
                delete m_depthCloudPoints;
            if (m_robotSurf)
                SDL_FreeSurface(m_robotSurf);
            SDL_Quit();
        }
        
        void display()
        {
            ROS_INFO("Starting display.");
            
            ros::Rate rate(10);
            while (ros::ok())
            {
                ros::spinOnce();
                updateDisplay();
                rate.sleep();
            }
        }

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "sensordisplay");
    ros::NodeHandle node;
    ROS_INFO("Initialized ROS.");

    SensorDisplay display(node);
    display.display();
    
    ROS_INFO("Bye!");
    return 0;
};

