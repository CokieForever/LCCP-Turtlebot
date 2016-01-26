#include "aruco/aruco.h"
#include "cv_bridge/cv_bridge.h"
#include <sensor_msgs/image_encodings.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
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
    cv::Mat deblurred_img;
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
    
    deblurred_img=deblurring(img).clone(); //////////////////////
    if(deblurred_img.empty())
    {	ROS_WARN ("Could not create the black and white image") ;
    }
    detector.detect(deblurred_img, markers);
    int nbMarkers = markers.size();
    ROS_INFO("Amount of markers: %d", nbMarkers);
    bool zoomed_pic=false;

    int k=0;
    int l=0;

    if (nbMarkers==0)
    {

          int width = deblurred_img.cols;
    	    int height = deblurred_img.rows;
    	    int channels = deblurred_img.channels();
    	    ROS_INFO("deblurred Image size: %dx%dx%d", width, height, channels);
    	    if (deblurred_img.cols <= 0 || deblurred_img.rows <= 0)
    		{
        	ROS_WARN("Received emtpy / unconventional image.");
        	return;
    		}
    

        cv::Size smallSize(width /2 ,height /2);
        cv::Mat dst;
        cv::Mat smallImages;


        for  ( int y =  0 ; y <deblurred_img.rows-smallSize.height+2 ; y += height/2 )
        {
          k=0;
          for  ( int x =  0 ; x <deblurred_img.cols-smallSize.width+2 ; x +=width/2 )
          {
	    ROS_INFO("no markers detected.Zoom in is the next step");
            zoomed_pic= true;
            cv::Rect rect = cv::Rect(x, y, smallSize.width, smallSize.height);
            cout << x << " " << y << " " << smallSize.width << " " << smallSize.height << endl;
            ROS_INFO("Image size: %dx%dx%dx%d", x, y, smallSize.width, smallSize.height);
            smallImages.push_back(cv::Mat(deblurred_img, rect));
            resize(cv::Mat(deblurred_img, rect), dst, deblurred_img.size(), 0, 0, CV_INTER_LINEAR);
	    cv::imshow("ZOOOOOOOOm", dst);
            detector.detect(dst,markers);
            cv::Mat frame = dst;
            drawingMarkers(frame, markers, k, l, zoomed_pic);
            k++;

          }
          l++;
      }
   }
   else
   {
      k=0;
      l=0;
      cv::Mat frame=deblurred_img;
      zoomed_pic=false;
      drawingMarkers(frame, markers, k, l, zoomed_pic );
   }



}

void DetectMarker::drawingMarkers(cv::Mat frame, std::vector<aruco::Marker> &markers, int k, int l, bool zoomed_pic)
{
	cv::Scalar colorScalar(255,155,0, 0);
	detect_marker::MarkersInfos markersInfos;
	int width = frame.cols;
    	int height = frame.rows;
    	int channels = frame.channels();
        int coord_division;
    	ROS_INFO("Image size2: %dx%dx%d", width, height, channels);
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
            detect_marker::MarkerInfo markerInfo;
            if (zoomed_pic)
            {
	      ROS_INFO(" Zoom");
              coord_division=2;
              cv::imshow("Marker Detection Zoomed", frame);
 
            }
            else
            {
              coord_division=1;
	      ROS_INFO("No Zoom");
              cv::imshow("Marker Detection Unzoomed", frame);
   
            }
              
              double xtrial=2*(center.x /(double)width)-1;
              double ytrial=2*(center.y/(double)height)-1;
	      markerInfo.x = 2*((center.x+k*width) /(coord_division*(double)width))-1; //tranformation of the center coordinates of zoomed pic to coordinates of initial pic
              markerInfo.y = 2*((center.y+l*height) /(coord_division*(double)height))-1;
              markerInfo.id = marker.id;
              markersInfos.infos.push_back(markerInfo);

            ROS_INFO("Marker %d: (%.3f, %.3f)", markerInfo.id, markerInfo.x, markerInfo.y);
            ROS_INFO("Zoomed Marker %d: (%.3f, %.3f)", markerInfo.id, xtrial, ytrial);
        }
    }
    cv::waitKey(0);
    
    ROS_INFO("Publishing %lu marker infos.", markersInfos.infos.size());
    m_markersPub.publish(markersInfos);


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

void DetectMarker::Detect()
{
    ROS_INFO("Starting detection.");
    ros::spin();
}



cv::Mat DetectMarker::deblurring(cv::Mat img )
{
     cv::Mat im;
     cv::Mat im_conv_kernel;
     cv::Mat im_correction;
     cv::Mat im_clone;
     cv::Mat im_new_est;
     cv::Mat dst;
     cv::Point anchor;
     double delta;
     int ddepth;
     int i;
     cv::cvtColor(img, im, CV_RGB2GRAY);


     cv::namedWindow("Image gray", CV_WINDOW_AUTOSIZE );
     cv::imshow("Image gray", im);
     
     anchor=cv::Point(-1,-1);
     delta = 0;
     ddepth = -1;
     int borderType=cv::BORDER_DEFAULT;
     int kernel_size = 13;
     int kernel_zeros_size= (int)(kernel_size/2); 

     //cv::Mat kernel_zeros= cv::Mat::zeros(1, kernel_zeros_size, CV_32F );
     cv::Mat kernel = cv::Mat::ones( 1,kernel_size, CV_32F )/ (float)(kernel_size);
    // cv::Mat kernel;
     //cv::hconcat(kernel_zeros, kernel_ones, kernel);

     // fk+1(x,y) = fk(x,y)
     im_clone= im.clone();

     for(i=0; i < 10; i++) {
     // Convolve f0(x,y)= g(x,y) with blur kernel
     // f0(x,y) ** kernel
     cv::filter2D(im_clone, im_conv_kernel, ddepth, kernel, anchor, delta, borderType );

     // Subtract from blurred image. Error correction = b(x,y) – ik(x,y) ** k(x.y)

     cv::subtract(im, im_conv_kernel, im_correction);
     // Add ik(x,y) with imCorrection – ik(x,y) + b(x,y) – ik(x,y) ** k(x,y)
     double lamda=0.1;
     cv::add(im_clone, lamda*im_correction,im_new_est);
     im_clone = im_new_est.clone();
     }
     cv::imshow("deblurred whole pic" ,im_clone);
     cv::threshold(im_clone, dst, 100 , 255, cv::THRESH_BINARY);
     cv::imshow("Black and white", dst);
     return dst;


}





