#include <sensor_msgs/image_encodings.h>

#include "detectmarker.h"
#include "detect_marker/MarkerInfo.h"
#include "detect_marker/MarkersInfos.h"

DetectMarker::DetectMarker(ros::NodeHandle& nodeHandle): m_nodeHandle(nodeHandle)
{
    ROS_INFO("Subscribing to camera image topic...");
    m_cameraSub = m_nodeHandle.subscribe("/camera/rgb/image_raw", 1, &DetectMarker::cameraSubCallback, this);
    ros::Rate loopRate(10);
    while (ros::ok() && m_cameraSub.getNumPublishers() <= 0)
        loopRate.sleep();

    ROS_INFO("Creating markers topic...");
    m_markersPub = m_nodeHandle.advertise<detect_marker::MarkersInfos>("/markerinfo", 10);
    ROS_INFO("Done, everything's ready.");
}

void DetectMarker::cameraSubCallback(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_INFO("Received image from camera.");

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
    
    aruco::MarkerDetector detector;
    std::vector<aruco::Marker> markers;
    
    detector.detect(img, markers);

    int nbMarkers = markers.size();
    ROS_INFO("Amount of markers: %d", nbMarkers);
    
    if (nbMarkers==0)
    {
        int width = img.cols;
        int height = img.rows;
        int channels = img.channels();
        ROS_INFO("Raw Image size: %dx%dx%d", width, height, channels);
        if (img.cols <= 0 || img.rows <= 0)
        {
            ROS_WARN("Received emtpy / unconventional image.");
            return;
        }
    
        cv::Size smallSize(width /2 ,height /2);
        cv::Mat dst;
        cv::Mat smallImages;
        for  ( int y =  0 ; y <img.rows-smallSize.height+2 ; y += height/2 )
        {
            for  ( int x =  0 ; x <img.cols-smallSize.width+2 ; x +=width/2 )
            {
                cv::Rect rect = cv::Rect(x, y, smallSize.width, smallSize.height);
                cout << x << " " << y << " " << smallSize.width << " " << smallSize.height << endl;
                ROS_INFO("Image size: %dx%dx%dx%d", x, y, smallSize.width, smallSize.height);
                smallImages.push_back(cv::Mat(img, rect));
                resize(cv::Mat(img, rect), dst, img.size(), 0, 0, CV_INTER_LINEAR); 
                detector.detect(dst,markers);
                cv::Mat frame = dst;
                drawingMarkers(frame, markers);
            }
        }
    }
    else
    {
        detector.detect(img,markers);
        cv::Mat frame=img;
        drawingMarkers(frame, markers);
    }

}

void DetectMarker::drawingMarkers(cv::Mat frame, std::vector<aruco::Marker> &markers )
{
    cv::Scalar colorScalar(255,155,0, 0);
    detect_marker::MarkersInfos markersInfos;
    int width = frame.cols;
    int height = frame.rows;
    int channels = frame.channels();
    ROS_INFO("Image size: %dx%dx%d", width, height, channels);
    if (frame.cols <= 0 || frame.rows <= 0)
    {
        ROS_WARN("Received emtpy / unconventional image.");
        return;
    }
    
    int nbMarkers=markers.size();
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
            double markerHeight = (std::max(corners[2].y, corners[3].y) - std::min(corners[0].y, corners[1].y)) / (double)height;
            detect_marker::MarkerInfo markerInfo;
            markerInfo.x = 2*(center.x/(double)width)-1;
            markerInfo.y = 2*(center.y/(double)height)-1;
            markerInfo.d = MARKER_REF_DIST / markerHeight;
            markerInfo.id = marker.id;
            markersInfos.infos.push_back(markerInfo);
            
            ROS_INFO("Marker %d: pos = (%.3f, %.3f); d = %.3f", markerInfo.id, markerInfo.x, markerInfo.y, markerInfo.d);
        }
    }
    
    ROS_INFO("Publishing %lu marker infos.", markersInfos.infos.size());
    m_markersPub.publish(markersInfos);

    cv::imshow("Marker Detection", frame);
    cv::waitKey(1);
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
}

bool DetectMarker::ComputerQuadrilateralCenter(Point points[4], Point *centerPoint)
{
    Point linePoints1[2] = {points[0], points[2]};
    Point linePoints2[2] = {points[1], points[3]};
    return ComputeLinesIntersection(linePoints1, linePoints2, centerPoint);
}

void DetectMarker::Filtering(const cv::Mat& img, cv::Mat** output)
{
    cv::Mat *field = NULL;
    Deinterlace(img, &field);

    //TODO

    *output = field;
}

void DetectMarker::Deinterlace(const cv::Mat& frame, cv::Mat** field1, cv::Mat** field2)
{
    int height = frame.rows;

    cv::Mat f1((height+1)/2, frame.cols, frame.type());
    cv::Mat f2((height+1)/2, frame.cols, frame.type());

    for (int y=0 ; y < height ; y+=2)
    {
        f1.row(y/2) = frame.row(y);
        if (field2 != NULL)
            f2.row(y/2) = frame.row(y+1);
    }

    *field1 = new cv::Mat(cv::Mat(frame.size(), frame.type()));
    cv::resize(f1, **field1, frame.size());

    if (field2 != NULL)
    {
        *field2 = new cv::Mat(cv::Mat(frame.size(), frame.type()));
        cv::resize(f2, **field2, frame.size());
    }
}

void DetectMarker::Detect()
{
    ROS_INFO("Starting detection.");
    ros::spin();
}
