#ifndef DETECTMARKER_H
#define DETECTMARKER_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "cv_bridge/cv_bridge.h"
#include "aruco/aruco.h"

class DetectMarker
{
    public:
        DetectMarker(ros::NodeHandle& nodeHandle);
        void Detect();

    private:
        struct Point
        {
            double x, y;
        };
        
        static const double MARKER_REF_DIST = 0.20;
        
        ros::NodeHandle& m_nodeHandle;
        ros::Subscriber m_cameraSub;
        ros::Publisher m_markersPub;

        void cameraSubCallback(const sensor_msgs::Image::ConstPtr& msg);
        bool ComputeLinesIntersection(Point linePoints1[2], Point linePoints2[2], Point *isectPoint);
        bool ComputeQuadrilateralCenter(Point points[4], Point *centerPoint);
        void drawingMarkers(cv::Mat frame, std::vector<aruco::Marker> &markers );
        
        void Deinterlace(const cv::Mat& frame, cv::Mat** field1, cv::Mat** field2=NULL);
        void Filtering(const cv::Mat& img, cv::Mat** output);
};

#endif // DETECTMARKER_H