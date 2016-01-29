#ifndef DETECTMARKER_H
#define DETECTMARKER_H
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "aruco/aruco.h"
#include "cv_bridge/cv_bridge.h"

class DetectMarker
{
    public:
        DetectMarker(ros::NodeHandle& nodeHandle);
        void detect();
        
    private:
        struct Point
        {
            double x, y;
        };

        static const double MARKER_REF_DIST = 0.20;

        static bool ComputeLinesIntersection(Point linePoints1[2], Point linePoints2[2], Point *isectPoint);
        static bool ComputeQuadrilateralCenter(Point points[4], Point *centerPoint);
        
        bool m_isRotating;
        ros::NodeHandle& m_nodeHandle;
        ros::Subscriber m_cameraSub;
        ros::Subscriber	m_velocitySub;
        ros::Publisher m_markersPub;
        
        std::vector<cv::Mat> splitImageAndZoom(cv::Mat& img, int nbBlocks, std::vector<int>& vecX, std::vector<int>& vecY);
        void velocityCallback(const geometry_msgs::Twist::ConstPtr& msg);
        void cameraSubCallback(const sensor_msgs::Image::ConstPtr& msg);
        void publishAndDrawMarkers(cv::Mat& frame, std::vector<aruco::Marker> &markers);
        cv::Mat deblurring(cv::Mat img);
        cv::Mat binarizeImage(cv::Mat& img, bool strong);
};

#endif // DETECTMARKER_H