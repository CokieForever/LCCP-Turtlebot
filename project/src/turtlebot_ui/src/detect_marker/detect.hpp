#include <ros/ros.h>
#include "detectmarker.h"
#include <QObject>
#include <QString>
/** @function main
int main( int argc, char** argv )
{
    ros::init(argc, argv, "detect_marker");
    ros::NodeHandle nodeHandle;
    ROS_INFO("Initialized ROS.");

    DetectMarker dm(nodeHandle);
    dm.Detect();
}
 */
class DetectMarkerWrapper : public QObject
{
  Q_OBJECT

public:
  DetectMarkerWrapper(ros::NodeHandle &nodeHandle);
  ~DetectMarkerWrapper();

public Q_SLOTS:
  void sl_call_gui(QString txt);

Q_SIGNALS:
  void si_call_gui(QString);
private:
  ros::NodeHandle m_nodeH;
  DetectMarker *dm;

};

