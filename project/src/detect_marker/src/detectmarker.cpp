#include <sensor_msgs/image_encodings.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "detectmarker.h"
#include <geometry_msgs/Twist.h>
#include "detect_marker/MarkerInfo.h"
#include "detect_marker/MarkersInfos.h"

DetectMarker::DetectMarker(ros::NodeHandle& nodeHandle): m_nodeHandle(nodeHandle)
{
    ROS_INFO("Subscribing to camera image topic...");
    m_cameraSub = m_nodeHandle.subscribe("/camera/rgb/image_raw", 1, &DetectMarker::cameraSubCallback, this);
    m_velocitySub = m_nodeHandle.subscribe("/mobile_base/commands/velocity", 1, &DetectMarker::velocityCallback, this);
    ros::Rate loopRate(10);
    rotation_speed=false;
    while (ros::ok() && m_cameraSub.getNumPublishers() <= 0)
        loopRate.sleep();

    ROS_INFO("Creating markers topic...");
    m_markersPub = m_nodeHandle.advertise<detect_marker::MarkersInfos>("/markerinfo", 10);
    ROS_INFO("Done, everything's ready.");
}



void DetectMarker::velocityCallback (const geometry_msgs::Twist::ConstPtr& msg)
{
	if (msg->angular.z !=0)
        {
	 rotation_speed=false;
        }
	else
	{
	  rotation_speed=true;
	}
	
}


void DetectMarker::cameraSubCallback(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_INFO("Received image from camera.");

    cv::Mat img;
    cv::Mat process_img;
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
   
    if (rotation_speed)
    {
	process_img=deblurring(img).clone();
    }
    else
    {
	process_img=img.clone();
    }

    if(process_img.empty())
    {    ROS_WARN ("Could not create the black and white image") ;
    }
    detector.detect(process_img, markers);
    int nbMarkers = markers.size();
    ROS_INFO("Amount of markers: %d", nbMarkers);
    bool zoomed_pic=false;
    
    int k=0;
    int l=0;
    if (nbMarkers==0)
    {

            int width = process_img.cols;
            int height = process_img.rows;
            int channels = process_img.channels();
            ROS_INFO("deblurred Image size: %dx%dx%d", width, height, channels);
            if (process_img.cols <= 0 || process_img.rows <= 0)
        {
            ROS_WARN("Received emtpy / unconventional image.");
            return;
        }
    
        cv::Size smallSize(width /2 ,height /2);
        cv::Mat dst;
        cv::Mat smallImages;


        for  ( int y =  0 ; y <process_img.rows-smallSize.height+2 ; y += height/2 )
        {
          k=0;
          for  ( int x =  0 ; x <process_img.cols-smallSize.width+2 ; x +=width/2 )
          {
            ROS_INFO("no markers detected.Zoom in is the next step");
            zoomed_pic= true;
            cv::Rect rect = cv::Rect(x, y, smallSize.width, smallSize.height);
            cout << x << " " << y << " " << smallSize.width << " " << smallSize.height << endl;
            ROS_INFO("Image size: %dx%dx%dx%d", x, y, smallSize.width, smallSize.height);
            smallImages.push_back(cv::Mat(process_img, rect));
            resize(cv::Mat(process_img, rect), dst, process_img.size(), 0, 0, CV_INTER_LINEAR);
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
      cv::Mat frame=process_img;
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
        
        if (!ComputeQuadrilateralCenter(corners, &center))
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

            double markerHeight = (std::max(corners[2].y, corners[3].y) - std::min(corners[0].y, corners[1].y)) / (double)(coord_division*height);
            double xtrial= 2*(center.x /(double)width)-1;
            double ytrial= 2*(center.y/(double)height)-1;
            markerInfo.x = 2*((center.x+k*width) /(coord_division*(double)width))-1; //tranformation of the center coordinates of zoomed pic to coordinates of initial pic
            markerInfo.y = 2*((center.y+l*height) /(coord_division*(double)height))-1;
            markerInfo.d = MARKER_REF_DIST / markerHeight;
            markerInfo.id = marker.id;
            markersInfos.infos.push_back(markerInfo);

            ROS_INFO("Marker %d: pos = (%.3f, %.3f); d = %.3f", markerInfo.id, markerInfo.x, markerInfo.y, markerInfo.d);
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

bool DetectMarker::ComputeQuadrilateralCenter(Point points[4], Point *centerPoint)
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



cv::Mat DetectMarker::deblurring(cv::Mat img )
{

   cv::Mat im;
   cv::cvtColor(img, im, CV_RGB2GRAY);
   int kernel_size=13;
   int size_zeros= 6;
   cv::Mat kernel;
   cv::Mat Y;
   cv::Mat kernel1= cv::Mat::zeros(1, size_zeros, CV_64F);
   cv::Mat kernel2=cv::Mat::ones(1, kernel_size-size_zeros, CV_64F)/(kernel_size-size_zeros);
   cv::hconcat(kernel1, kernel2, kernel);
   cv::Mat J1;
   cv::Mat J2;
   cv::Mat J3;
   cv::Mat J4;
   cv::Mat wI;
   cv::Mat anEstimate;
   cv::Mat anEstimate1;
   cv::Mat Kernel_values= cv::Mat(1, kernel_size-size_zeros, CV_64F);
   cv::Mat zeros_add= cv::Mat::zeros(1, im.cols-kernel_size+size_zeros, CV_64F);
   cv::Mat kernel_zeros_concat;
   cv::Mat H=cv::Mat::zeros(im.rows, im.cols, CV_64F);
   cv::Mat Hdft, Hdft_conj, Ydft, anEstimate_dft, HYdft, HdftAnEstimdft= cv::Mat(im.rows, im.cols, CV_64FC2);
   cv::Mat HYidft;
   cv::Mat lambda2;
   cv::Mat planes[2];
   cv::Mat H_anEst_inv;
   double eps=std::numeric_limits<double>::epsilon();
   int r;
   int i;
   double lambda=0;
   
   J4=cv::Mat::zeros(im.rows*im.cols, 2, CV_64F);
   J3=cv::Mat::zeros(im.rows, im.cols, CV_64F);

   im.convertTo(im, CV_64F, 1.0/255);
   J1=im.clone();
   J2=im.clone();
   wI=im.clone();

   //create an array with all the non-zero values of the kernel in their normal order
   for (r=0; r<kernel_size-size_zeros; r++)
   {
    Kernel_values.at<double>(r)=kernel.at<double>(kernel_size-size_zeros+r-1);
   }


    //concatenate the non-tero values with the zeros in order to crate a row (1*im.cols)
    cv::hconcat(Kernel_values, zeros_add, kernel_zeros_concat);
    //first elements of H have the non-zero values of the kernel. The rest values are zero
    H.row(0)=H.row(0)+kernel_zeros_concat;
    //Discrete Fourier Transform
    cv::dft(H, Hdft, cv::DFT_COMPLEX_OUTPUT);
   //deblurring through ten iterations
   //J1 is the original grayscale image, J2 is the result of each iteration, J3 is the result of the previous iteration  
   for(i=0;i<10;i++)
  {

     if (i>1)
    {
      ROS_INFO("BEFORE");
      double A=J4.col(0).dot(J4.col(1));
      double B=J4.col(1).dot(J4.col(1))+eps;
      lambda=A/B;
      lambda = max(min(lambda,(double)1),(double)0);
    }

     
     Y = cv::max(J2 + lambda*(J2 - J3),(double)0);
     cv::dft(Y, Ydft, cv::DFT_COMPLEX_OUTPUT);
     //multiplication of Fourier transformations
     cv::mulSpectrums(Hdft, Ydft, HYdft, 0);
    //inverse Fourier transformation
     cv::idft(HYdft, HYidft, cv::DFT_SCALE + cv::DFT_REAL_OUTPUT);
    //set to eps when zero, to erase problems with the following divisions
     HYidft.setTo(eps, HYidft==0);
     cv::divide(wI, HYidft, anEstimate1, 1);
     anEstimate= anEstimate1+eps;
     cv::dft(anEstimate, anEstimate_dft, cv::DFT_COMPLEX_OUTPUT);
     J3=J2.clone();
     //conj calculation
     cv::split(Hdft, planes);
     planes[1]=-planes[1];
     cv::merge(planes, 2 , Hdft_conj);
     
     cv::mulSpectrums(Hdft_conj, anEstimate_dft, HdftAnEstimdft, 0);
     cv::idft(HdftAnEstimdft, H_anEst_inv, cv::DFT_SCALE + cv::DFT_REAL_OUTPUT);
     J2=cv::max(Y.mul(H_anEst_inv), 0);;
     J4.col(0).copyTo(J4.col(1));
     cv::Mat J2_Y= J2-Y;
     J2_Y.reshape(0, im.rows*im.cols).copyTo(J4.col(0));

  }
     cv::Mat dst;
     cv::imshow("Black and white1", J2);
     J2.convertTo(J2, CV_8U, 255.0);
     //recreate bit-image according to the threshold, keep black and white values to improve detection 
     cv::threshold(J2, dst, 170 , 250, cv::THRESH_BINARY);
     cv::imshow("Black and white", dst);
     cv::waitKey(0);
     return J2;
}



