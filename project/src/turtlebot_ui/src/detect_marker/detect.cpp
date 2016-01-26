#include "detect.hpp"
DetectMarkerWrapper::DetectMarkerWrapper(ros::NodeHandle &nodeHandle) : m_nodeH(nodeHandle)
{
  dm = new DetectMarker(m_nodeH);
  connect(dm, SIGNAL(si_status_togui(QString)), this, SLOT(sl_call_gui(QString)));
}
void DetectMarkerWrapper::sl_call_gui(QString txt)
{
  si_call_gui(txt);
  dm->Detect();

}

DetectMarkerWrapper::~DetectMarkerWrapper()
{
  delete this;
}
