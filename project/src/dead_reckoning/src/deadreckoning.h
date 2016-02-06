#ifndef DEADRECKONING_H
#define DEADRECKONING_H

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
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include "dead_reckoning/Grid.h"
#include "detect_marker/MarkerInfo.h"
#include "detect_marker/MarkersInfos.h"
#include "detect_friend/Friend_id.h"
#include "detect_friend/FriendsInfos.h"
#include "sdl_gfx/SDL_rotozoom.h"

class Grid
{
    public:
        struct ProbabilisticPoint
        {
            double x, y;
            ros::Time t;
            double p;
        };

        Grid(double precision=0.05, ros::Duration ttl=ros::Duration(120.0), double minX=-10, double maxX=10, double minY=-10, double maxY=10, bool resizeable=true);
        Grid(const Grid& grid);
        Grid& operator=(const Grid& grid);
        ~Grid();
        
        double precision() const;
        double minX() const;
        double minY() const;
        bool addPoint(double x, double y, ros::Time t, double p);
        bool addPoint(ProbabilisticPoint point);
        double get(double x, double y);
        double* getAll(int* width=NULL, int *height=NULL, double *scale=NULL) const;
        SDL_Surface* draw(int w, int h, double minX, double maxX, double minY, double maxY, SDL_Surface *surf=NULL);
    
    private:
        double m_precision;
        ros::Duration m_ttl;
        double m_minX, m_maxX, m_minY, m_maxY;
        ProbabilisticPoint** m_data;
        int m_height, m_width;
        bool m_resizeable;
        
        void init();
        void empty();
        void updateSize();
        double _get(int ix, int iy);
};

class DeadReckoning
{
    private:
        struct Vector
        {
            double x, y;
        };
        
        struct StampedPos
        {
            double x, y, z;
            ros::Time t;
        };

        static const double ANGLE_PRECISION;  //deg
        static const int NB_CLOUDPOINTS;
        static const double MAX_RANGE;
        static const int SCREEN_HEIGHT;
        static const int SCREEN_WIDTH;
        static const std::string LOCALMAP_SCAN_TRANSFORM_NAME;
        static const std::string LOCALMAP_DEPTH_TRANSFORM_NAME;
        static const std::string ROBOTPOS_TRANSFORM_NAME;
        static const std::string SCANGRIDPOS_TRANSFORM_NAME;
        static const std::string DEPTHGRIDPOS_TRANSFORM_NAME;
        static const std::string MARKERPOS_TRANSFORM_NAME;
        static const std::string FRIENDPOS_TRANSFORM_NAME;
        static const int SIZE_POSITIONS_HIST;
        static const int NB_FRIENDS;
        
        static double modAngle(double rad);
        static void pointCloudToLaserScan(const sensor_msgs::PointCloud2ConstPtr &cloud_msg, sensor_msgs::LaserScan& output);
        static SDL_Surface* loadImg(std::string path);
        
        ros::NodeHandle& m_node;
        ros::Subscriber m_orderSub;
        ros::Subscriber m_odomSub;
        ros::Subscriber m_laserSub;
        ros::Subscriber m_depthSub;
        ros::Subscriber m_imuSub;
        ros::Subscriber m_localMapScanSub, m_localMapDepthSub;
        ros::Subscriber m_markersSub, m_friendsSub;
        ros::Publisher m_laserScanPub, m_laserDepthPub;
        ros::Publisher m_scanGridPub, m_depthGridPub;
        double *m_scanRanges, *m_depthRanges;
        bool m_simulation;
        ros::Time m_time;
        StampedPos m_position;
        StampedPos *m_positionsHist;
        int m_positionsHistIdx;
        double m_offsetX, m_offsetY, m_offsetZ, m_offsetZOdom;
        double m_linearSpeed;
        double m_angularSpeed;
        Vector *m_scanCloudPoints, *m_depthCloudPoints;
        int m_scanCloudPointsStartIdx, m_depthCloudPointsStartIdx;
        Grid m_scanGrid, m_depthGrid;
        SDL_Surface *m_screen;
        SDL_Surface *m_robotSurf, *m_markerSurf, *m_markerSurfTransparent;
        SDL_Surface **m_friendSurf, **m_friendSurfTransparent;
        SDL_Surface *m_gridSurf;
        double m_minX, m_maxX, m_minY, m_maxY;
        tf::TransformBroadcaster m_transformBroadcaster;
        StampedPos m_markersPos[256];
        bool m_markerInSight[256];
        StampedPos *m_friendsPos;
        bool *m_friendInSight;
        
        StampedPos getPosForTime(const ros::Time& time);
        void friendsCallback(const detect_friend::FriendsInfos::ConstPtr& friendsInfos);
        void markersCallback(const detect_marker::MarkersInfos::ConstPtr& markersInfos);
        void IMUCallback(const sensor_msgs::Imu::ConstPtr& imu);
        void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);
        void moveOrderCallback(const geometry_msgs::Twist::ConstPtr& order);
        void localMapScanCallback(const nav_msgs::OccupancyGrid::ConstPtr& occ);
        void localMapDepthCallback(const nav_msgs::OccupancyGrid::ConstPtr& occ);
        void updateGridFromOccupancy(const nav_msgs::OccupancyGrid::ConstPtr& occ, Grid& grid);
        void processLaserScan(sensor_msgs::LaserScan& scan, bool invert, double *ranges, Vector *cloudPoints, int& startIdx);
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
        void depthCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);
        void publishTransforms();
        void publishMarkersTransforms();
        void publishFriendsTransforms();
        void publishGrid(const Grid& grid, ros::Publisher& pub);
        bool initSDL();
        void updateDisplay();
        void convertPosToDisplayCoord(double fx, double fy, int& x, int& y);

    public:
        DeadReckoning(ros::NodeHandle& node, bool simulation=true, double minX=-5, double maxX=5, double minY=-5, double maxY=5);
        ~DeadReckoning();
        void reckon();
};

#endif