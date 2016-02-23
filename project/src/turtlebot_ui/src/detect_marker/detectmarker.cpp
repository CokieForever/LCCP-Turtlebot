#include "detectmarker.h"

DetectMarker::DetectMarker(ros::NodeHandle& nodeHandle): m_nodeHandle(nodeHandle)
{
  //
}
void DetectMarker::Start()
{
    si_status_togui("Subscribing to image topic: Waiting for publisher...");
    cout << "before signal" << endl;
    m_cameraSub = m_nodeHandle.subscribe("/camera/rgb/image_rect_color", 1, &DetectMarker::cameraSubCallback, this);
    ros::Rate loopRate(10);

  while (ros::ok() && m_cameraSub.getNumPublishers() <= 0)
        loopRate.sleep();

    //emit signal that a publisher has been found
  si_status_togui("Creating markers topic...");
    m_markersPub = m_nodeHandle.advertise<turtlebot_ui::MarkersInfos>("/markerinfo", 10);

    /*while (ros::ok() && m_markersPub.getNumSubscribers() <= 0)
        loopRate.sleep();*/

    si_status_togui("Done, everything's ready!");
}

void DetectMarker::cameraSubCallback(const sensor_msgs::ImageConstPtr& msg)
{
    //ROS_INFO("Received image from camera.");

    cv::Mat img;
    try
    {
        cv_bridge::CvImageConstPtr imgPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        if (imgPtr == NULL)
        {
            ROS_WARN("Received NULL image.");
            return;
        }
        img = imgPtr->image;
    }
    catch (cv_bridge::Exception& ex)
    {
        ROS_ERROR("cv_bridge exception: %s", ex.what());
        exit(-1);
    }

    int width = img.cols;
    int height = img.rows;
    int channels = img.channels();
    //ROS_INFO("Image size: %dx%dx%d", width, height, channels);
    if (img.cols <= 0 || img.rows <= 0)
    {
        ROS_WARN("Received emtpy / unconventional image.");
        return;
    }
    
    aruco::MarkerDetector detector;
    std::vector<aruco::Marker> markers;
    
    cv::Scalar colorScalar(255,155,0, 0);
    detector.detect(img, markers);

    int nbMarkers = markers.size();
    //ROS_INFO("Amount of markers: %d", nbMarkers);

    turtlebot_ui::MarkersInfos markersInfos;
    cv::Mat frame = img;
    for (int i=0 ; i < nbMarkers ; i++)
    {
        aruco::Marker& marker = markers[i];
        marker.draw(frame, colorScalar);

        Point center;
        Point corners[4];
        for (int j=0 ; j < 4 ; j++)
        {
            corners[j].x = marker[j].x;
            corners[j].y = marker[j].y;
        }
        
        if (!ComputerQuadrilateralCenter(corners, &center))
            ROS_WARN("Unable to compute center.");
        else
        {
            turtlebot_ui::MarkerInfo markerInfo;
            markerInfo.x = 2*(center.x/(double)width)-1;
            markerInfo.y = 2*(center.y/(double)height)-1;
            markerInfo.id = marker.id;
            markersInfos.infos.push_back(markerInfo);

            ROS_INFO("Marker %d: (%.3f, %.3f)", markerInfo.id, markerInfo.x, markerInfo.y);
        }
    }
    
    //ROS_INFO("Publishing %lu marker infos.", markersInfos.infos.size());
    if(markersInfos.infos.size()) m_markersPub.publish(markersInfos);

    si_togui_frame(frame);
   // cv::imshow("Marker Detection", frame);

    cv::waitKey(1);
    ros::spinOnce();
}

bool DetectMarker::ComputeLinesIntersection(Point linePoints1[2], Point linePoints2[2], Point *isectPoint)
{
    double x1 = linePoints1[0].x;
    double x2 = linePoints1[1].x;
    double y1 = linePoints1[0].y;
    double y2 = linePoints1[1].y;    

    double x3 = linePoints2[0].x;
    double x4 = linePoints2[1].x;
    double y3 = linePoints2[0].y;
    double y4 = linePoints2[1].y;    

    double d = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4);
    if (fabs(d) < 1e-6)
        return false;

    isectPoint->x = ((x1*y2 - y1*x2) * (x3-x4) - (x1-x2) * (x3*y4 - y3*x4)) / d;
    isectPoint->y = ((x1*y2 - y1*x2) * (y3-y4) - (y1-y2) * (x3*y4 - y3*x4)) / d;
    return true;
    ros::spinOnce();
}

bool DetectMarker::ComputerQuadrilateralCenter(Point points[4], Point *centerPoint)
{
    Point linePoints1[2] = {points[0], points[2]};
    Point linePoints2[2] = {points[1], points[3]};
    return ComputeLinesIntersection(linePoints1, linePoints2, centerPoint);
}

void DetectMarker::Detect()
{

    ROS_INFO("Starting detection");
    ros::spin();
}
