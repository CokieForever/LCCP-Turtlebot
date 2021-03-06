#ifndef DETECTMARKER_H
#define DETECTMARKER_H

#include <ros/ros.h>
#include "aruco/aruco.h"
#include "cv_bridge/cv_bridge.h"
#include <sensor_msgs/image_encodings.h>

#include <sensor_msgs/Image.h>
#include <QString>
#include <QObject>

#include "turtlebot_ui/MarkerInfo.h"
#include "turtlebot_ui/MarkersInfos.h"
#include <iostream>
/**
 * @brief DetectMarker class is modified in order to work with QT.
 * Every output can be sent to the user interface via signals.
 */
class DetectMarker : public QObject
{
  Q_OBJECT

    public:
        DetectMarker(ros::NodeHandle& nodeHandle );
        void Start();
        void Detect();

    Q_SIGNALS:
        void si_status_togui(QString); ///< signal to send the status message to the UI
        void si_togui_frame(cv::Mat &); ///< signal to send the image frames to the UI
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
        void gui_print(QString text);
};

#endif // DETECTMARKER_H
