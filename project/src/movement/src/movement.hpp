#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "detect_marker/MarkersInfos.h"
#include "kobuki_msgs/BumperEvent.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Bool.h"
/**
 * @class Mover
 * @brief This class is used for the movement of the robot.
 */
class Mover
{
public:
  const static double FORWARD_SPEED_MPS = 0.2; ///< Tunable paramter: forward speed (m/s)
  const static double MIN_SCAN_ANGLE_RAD = -0.57 + M_PI; ///< remove M_PI for gazebo
  const static double MAX_SCAN_ANGLE_RAD = 0.57 + M_PI; ///< remove M_PI for gazebo
  const static float  MIN_PROXIMITY_RANGE_M = 0.7;	///<  Should be smaller than sensor_msgs::LaserScan::range_max
  Mover();
  void startMoving();
  tf::TransformListener listener; ///< Tf listener
private:
    ros::NodeHandle	node; ///< Node handler
    ros::Publisher commandPub;	///< Publisher to the robot's velocity command topic
    ros::Publisher targetReachedRequest; ///< Publishes a "true" state
    ros::Subscriber turtleStar; ///< Subscriber to the /gotStar topic
    ros::Subscriber	laserSub;	///< Subscriber to the robot's laser scan topic
    //ros::Subscriber markerId;   // Subscriber to the detect_markers topic including the founded aruco-markers
    ros::Subscriber bumperSub;     ///< Subscriber to the robot's bumber event topic
    ros::Subscriber getLocationSub; ///< Subscriber to the /markerinfo topic
    ros::Subscriber targetFinishedSub; ///< Subscriber to check whether the search is finished
    tf::TransformListener m_coordinateListener; ///< Tf listener for the coordinates of the robot/goal marker
    double m_turtleSpeed; ///< double that saves the speed of the bot
    bool m_outOfSight; ///< set this attribute if turtlebot got target, but it went out of sight
    bool m_finishedMarkerSearch; ///< flag that indicates that the search is finished
    int m_searchMarker; ///< the marker that needs to be approached next
    float m_distanceToMarker; ///< actual distance to marker
    float m_angularVelocity; ///< angular velocity
    bool m_keepMoving; ///< Indicates whether the robot should continue moving
    bool m_reachedTarget; ///< Set true if marker is reached
    bool m_gotTarget; ///< Set true if Mover got a target marker from the sensor node
    bool m_driveNow; ///< Set true if there is no obstacle in the way
    geometry_msgs::Vector3Stamped m_target; ///< vector position of the target
    int  m_nextId; ///< saves the next marker id that needs to be approached
    void moveRandomly();
    void starCallBack(const std_msgs::Bool::ConstPtr &star);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    void bumperSubCallback(const kobuki_msgs::BumperEvent::ConstPtr& bumper_msg);
    void approachMarker(int param_marker_id);
    void getLocationCallback(const detect_marker::MarkersInfos::ConstPtr &marker_msg);
    void targetFinishedCallback(const std_msgs::EmptyConstPtr empty);
    void driveForwardOdom(double distance);
    void rotateOdom(double angle);
};
