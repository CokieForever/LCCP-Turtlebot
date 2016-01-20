#include "ros/ros.h"
#include "../../detect_marker/src/detectmarker.h"
#include "sensor_msgs/LaserScan.h"
#include "detect_marker/MarkerInfo.h"
#include "detect_marker/MarkersInfos.h"

class initial_detection {
public:
    // Tunable parameters
    const static double FORWARD_SPEED_MPS = 0.5;
    const static double ROTATION_SPEED = 0.8;
    const static double MIN_SCAN_ANGLE_RAD = -30.0/180*M_PI;
    const static double MAX_SCAN_ANGLE_RAD = +30.0/180*M_PI;
    const static double ROTATION_ANGLE = +60.0/180*M_PI;
    const static float MIN_PROXIMITY_RANGE_M = 0.75;	// Should be smaller than sensor_msgs::LaserScan::range_max
    const static double ANGLE_PRECISION = 1.0; //deg
    
    initial_detection();
    ~initial_detection();
    void startRotating();

private:
    static double modAngle(double rad);

    ros::NodeHandle	m_node;
    ros::Publisher	m_commandPub;   // Publisher to the robot's velocity command topic
    ros::Subscriber	m_markerSub;    //Subscriber to the detected markers topic
    ros::Subscriber m_scanSub;      // Subscriber to the robot's laser scan topic
    int m_next_id;
    bool m_next_id_detected;
    double m_angularSpeed, m_linearSpeed;
    bool m_obstacle;
    double *m_ranges;

    void detectCallback(const detect_marker::MarkersInfos::ConstPtr& detected_Marker);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    bool proximityAlert();
};
