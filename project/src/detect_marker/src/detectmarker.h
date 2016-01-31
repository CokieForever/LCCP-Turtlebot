#ifndef DETECTMARKER_H
#define DETECTMARKER_H
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include "aruco/aruco.h"
#include "cv_bridge/cv_bridge.h"

class DetectMarker
{
    public:
        static const double MARKER_REF_DIST = 480 * 0.2 / 0.175;
        static const double MARKER_SIZE = 0.175;
        
        DetectMarker(ros::NodeHandle& nodeHandle);
        void detect();
        
    private:
        struct Point
        {
            double x, y;
        };
        
        static bool ComputeLinesIntersection(Point linePoints1[2], Point linePoints2[2], Point *isectPoint);
        static bool ComputeQuadrilateralCenter(Point points[4], Point *centerPoint);
        
        bool m_isRotating;
        ros::NodeHandle& m_nodeHandle;
        ros::Subscriber m_cameraSub;
        ros::Subscriber	m_IMUSub;
        ros::Publisher m_markersPub;
        
        std::vector<cv::Mat> splitImageAndZoom(cv::Mat& img, int nbBlocks, std::vector<int>& vecX, std::vector<int>& vecY);
        void IMUCallback(const sensor_msgs::Imu::ConstPtr& imu);
        void cameraSubCallback(const sensor_msgs::Image::ConstPtr& msg);
        void publishAndDrawMarkers(cv::Mat& frame, std::vector<aruco::Marker> &markers, ros::Time time=ros::Time::now());
        cv::Mat deblurring(cv::Mat img);
        cv::Mat binarizeImage(cv::Mat& img, bool strong);
};

#endif // DETECTMARKER_H