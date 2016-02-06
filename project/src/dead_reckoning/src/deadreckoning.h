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

/**
 * \class Grid
 * \brief Represents the discretized world as a grid containing probabilities of obstacles.
 */
class Grid
{
    public:
        /**
         * \struct ProbabilisticPoint
         * \brief A point in the grid, with time stamp and probability of obstacle.
         */
        struct ProbabilisticPoint
        {
            double x;       /*!< x-coordinate of the point in the world, not discretized. */
            double y;       /*!< y-coordinate of the point in the world, not discretized. */
            ros::Time t;    /*!< Time of the last update of this point. */
            double p;       /*!< Probability of the presence of an obstacle, between 0 and 1. */
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
        double m_precision;             /*!< Precision of the grid (m / unit). */
        ros::Duration m_ttl;            /*!< Time To Live of the points inside the grid. */
        double m_minX;                  /*!< x-coordinate of the upper-left corner of the grid in the real world. */
        double m_maxX;                  /*!< x-coordinate of the lower-right corner of the grid in the real world. */
        double m_minY;                  /*!< y-coordinate of the upper-left corner of the grid in the real world. */
        double m_maxY;                  /*!< y-coordinate of the lower-right corner of the grid in the real world. */
        ProbabilisticPoint** m_data;    /*!< Raw data of the grid, as a 1D array of pointers. */
        int m_height;                   /*!< Height of the grid (units). */
        int m_width;                    /*!< Width of the grid (units). */
        bool m_resizeable;              /*!< Indicated if the grid can be dynamically resized or not. */
        
        void init();
        void empty();
        void updateSize();
        double _get(int ix, int iy);
};

/**
 * \class DeadReckoning
 * \brief Main class handling the dead reckoning and the odometry of the robot.
 *
 * This class is responsible for estimating the robot's position in the real world according to information provided
 * by the robot's sensors, including wheel sensors, laser scan, inertial sensors, depth images, etc.
 * It will also estimate the position of obstacles, markers and friends, and will keep and provide a map of their positions
 * even if they are not in sight anymore, thus allowing a live build of a world full map.
 * Positions of the robot, the markers and the friends are published via Transformations and the map via a custom message.
 */
class DeadReckoning
{
    private:
        /**
         * \struct Vector
         * \brief A 2D velocity vector, can also be used as a 2D point.
         */
        struct Vector
        {
            double x;   /*!< x-coordinate of the point. */
            double y;   /*!< y-coordinate of the point. */
        };
        
        /**
         * \struct StampedPos
         * \brief A 2D position with orientation (z-axis) information and a time stamp.
         */
        struct StampedPos
        {
            double x;       /*!< x-coordinate of the point. */
            double y;       /*!< y-coordinate of the point. */
            double z;       /*!< Rotation of the point around the z-axis. */
            ros::Time t;    /*!< Time stamp of the point. */
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
        
        ros::NodeHandle& m_node;                            /*!< Main node handle. */
        ros::Subscriber m_orderSub;                         /*!< Subscriber to the robot's orders (mobile_base/commands/velocity). */
        ros::Subscriber m_odomSub;                          /*!< Subscriber to the robot's odometry (/odom). */
        ros::Subscriber m_laserSub;                         /*!< Subscriber to the robot's laser scan (/scan). */
        ros::Subscriber m_depthSub;                         /*!< Subscriber to the robot's unregistered depth image (/camera/depth/points). */
        ros::Subscriber m_imuSub;                           /*!< Subscriber to the robot's filtered IMU information (/mobile_base/sensors/imu_data). */
        ros::Subscriber m_localMapScanSub;                  /*!< Subscriber to the local map of the robot, provided by the local_map node based on laser scan data (/local_map_scan/local_map). */
        ros::Subscriber m_localMapDepthSub;                 /*!< Subscriber to the local map of the robot, provided by the local_map node based on depth image scan data (/local_map_depth/local_map). */
        ros::Subscriber m_markersSub;                       /*!< Subscriber to the markers information, provided by the detect_marker node based on camera data (/markerinfo). */
        ros::Subscriber m_friendsSub;                       /*!< Subscriber to the friends information, provided by the detect_friend node based on camera data (/friendinfo). */
        ros::Publisher m_laserScanPub;                      /*!< Publisher of the filtered laser scan for the local_map node (/local_map_scan/scan). */
        ros::Publisher m_laserDepthPub;                     /*!< Publisher of the depth image as a laser scan for the local_map node (/local_map_depth/scan). */
        ros::Publisher m_scanGridPub;                       /*!< Publisher of the map created from laser scan data (/dead_reckoning/scan_grid). */
        ros::Publisher m_depthGridPub;                      /*!< Publisher of the map created from depth image data (/dead_reckoning/depth_grid). */
        double *m_scanRanges;                               /*!< Buffer of the last 360° known scan ranges, especially useful when dealing with a non 360° laser scan. */
        double *m_depthRanges;                              /*!< Buffer of the last 360° known ranges, computed from depth image data. */
        bool m_simulation;                                  /*!< Indicates if we run in simulation mode or not. */
        StampedPos m_position;                              /*!< Last estimation of the robot's position in the real world. */
        StampedPos *m_positionsHist;                        /*!< History of estimations of the robot's position in the real world. */
        int m_positionsHistIdx;                             /*!< Start index for the position estimations history. */
        double m_offsetX;                                   /*!< Offset along the x-axis between the internal coordinate system and the robot's odometry coordinates system. */
        double m_offsetY;                                   /*!< Offset along the y-axis between the internal coordinate system and the robot's odometry coordinates system. */
        double m_offsetZ;                                   /*!< Orientation offset between the internal coordinate system and the robot's IMU coordinates system. */
        double m_offsetZOdom;                               /*!< Orientation offset between the internal coordinate system and the robot's odometry coordinates system. */
        double m_linearSpeed;                               /*!< Last linear velocity order sent to the robot. */
        double m_angularSpeed;                              /*!< Last angular velocity order sent to the robot. */
        Vector *m_scanCloudPoints;                          /*!< Cloud points, in the real world, representing the laser scan data. */
        Vector *m_depthCloudPoints;                         /*!< Cloud points, in the real world, representing the depth image data. */
        int m_scanCloudPointsStartIdx;                      /*!< Start index for the laser scan cloud points. */
        int m_depthCloudPointsStartIdx;                     /*!< Start index for the depth image cloud points. */
        Grid m_scanGrid;                                    /*!< Current map of the world built from laser scan data. */
        Grid m_depthGrid;                                   /*!< Current map of the world built from depth image data. */
        SDL_Surface *m_screen;                              /*!< Main display surface. */
        SDL_Surface *m_robotSurf;                           /*!< Internal bitmap used to draw the robot. */
        SDL_Surface *m_markerSurf;                          /*!< Internal bitmap used to draw a marker. */
        SDL_Surface *m_markerSurfTransparent;               /*!< Internal bitmap used to draw a half-transparent marker. */
        SDL_Surface **m_friendSurf;                         /*!< Internal bitmap used to draw a friend. */
        SDL_Surface **m_friendSurfTransparent;              /*!< Internal bitmap used to draw a half-transparent friend. */
        SDL_Surface *m_gridSurf;                            /*!< Internal bitmap on which the map is drawn. */
        double m_minX;                                      /*!< x-coordinate of the upper-left corner of the display in the real world. */
        double m_maxX;                                      /*!< x-coordinate of the lower-right corner of the display in the real world. */
        double m_minY;                                      /*!< y-coordinate of the upper-left corner of the display in the real world. */
        double m_maxY;                                      /*!< y-coordinate of the lower-right corner of the display in the real world. */
        tf::TransformBroadcaster m_transformBroadcaster;    /*!< Main transformation broadcaster. */
        StampedPos m_markersPos[256];                       /*!< Last known positions of all markers (IDs from 0 to 255).*/
        bool m_markerInSight[256];                          /*!< Indicates which markers are still in sight (IDs from 0 to 255).*/
        StampedPos *m_friendsPos;                           /*!< Last known positions of all friends.*/
        bool *m_friendInSight;                              /*!< Indicates which friends are still in sight.*/
        
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