#ifndef DETECTMARKER_H
#define DETECTMARKER_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

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
        ros::NodeHandle& m_nodeHandle;
        ros::Subscriber m_cameraSub;
        ros::Publisher m_markersPub;

        void cameraSubCallback(const sensor_msgs::Image::ConstPtr& msg);
        bool ComputeLinesIntersection(Point linePoints1[2], Point linePoints2[2], Point *isectPoint);
        bool ComputerQuadrilateralCenter(Point points[4], Point *centerPoint);
};

#endif // DETECTMARKER_H
