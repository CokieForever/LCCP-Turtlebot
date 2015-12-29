#include "detectmarker.h"

DetectMarker::DetectMarker()
{
    pub = nh.advertise<detect_marker::Markers>("/int8_marker_ids", 10);
    sub =  nh.subscribe("/camera/rgb/image_rect_color", 1, &DetectMarker::rgbCallback, this);
}

void DetectMarker::publishMarker()
{
    pub.publish(ui8_markers);
}

void DetectMarker::rgbCallback(const sensor_msgs::ImageConstPtr& msg)
{

   // cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
      DetectMarker::cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& ex)
    {
        ROS_ERROR("cv_bridge exception: %s", ex.what());
        exit(-1);
    }
    aruco::MarkerDetector detector;
    std::vector<aruco::Marker> detMarker;

    detector.detect(cv_ptr->image,detMarker);

    ROS_INFO("Amount of markers: %ld", detMarker.size());

    for (int i=0; i<detMarker.size(); i++)
         ui8_markers.marker[i] = detMarker[i].id;

    cv::imshow("Marker Detection", cv_ptr->image);
    cv::waitKey(30); //!!!!!!
}
void DetectMarker::Detection()
{
    ros::Rate   rate(100);
    while (ros::ok())
    {
        publishMarker();
        ros::spinOnce();
        rate.sleep();
    }
}
