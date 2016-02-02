#include <sensor_msgs/image_encodings.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "detectmarker.h"
#include <geometry_msgs/Twist.h>
#include "detect_marker/MarkerInfo.h"
#include "detect_marker/MarkersInfos.h"

DetectMarker::DetectMarker(ros::NodeHandle& nodeHandle): m_nodeHandle(nodeHandle), m_isRotating(false)
{
    ros::Rate loopRate(10);
    
    ROS_INFO("Subscribing to camera image topic...");
    m_cameraSub = m_nodeHandle.subscribe("/camera/rgb/image_raw", 1, &DetectMarker::cameraSubCallback, this);
    while (ros::ok() && m_cameraSub.getNumPublishers() <= 0)
        loopRate.sleep();
    
    ROS_INFO("Subscribing to robot IMU...");
    m_IMUSub = m_nodeHandle.subscribe("/mobile_base/sensors/imu_data", 1000, &DetectMarker::IMUCallback, this);
    while (ros::ok() && m_IMUSub.getNumPublishers() <= 0)
        loopRate.sleep();    

    ROS_INFO("Creating markers topic...");
    m_markersPub = m_nodeHandle.advertise<detect_marker::MarkersInfos>("/markerinfo", 10);
    
    ROS_INFO("Done, everything's ready.");
}

void DetectMarker::IMUCallback(const sensor_msgs::Imu::ConstPtr& imu)
{
    m_isRotating = fabs(imu->angular_velocity.z) > 0.4;
}

std::vector<cv::Mat> DetectMarker::splitImageAndZoom(cv::Mat& img, int nbBlocks, std::vector<int>& vecX, std::vector<int>& vecY)
{
    std::vector<cv::Mat> vec;
    double deltaX = img.cols / (double)nbBlocks;
    double deltaY = img.rows / (double)nbBlocks;
    for (double x = 0 ; x < img.cols ; x += deltaX)
    {
        int ix = ceil(x);
        for (double y = 0 ; y < img.rows ; y += deltaY)
        {
            int iy = ceil(y);
            cv::Mat tile = img(cv::Range(iy, min((int)floor(y+deltaY)+1, img.rows)), cv::Range(ix, min((int)floor(x+deltaX)+1, img.cols)));
            cv::Mat resizedTile ;
            cv::resize(tile, resizedTile, img.size(), 0, 0, CV_INTER_NN);
            
            vecX.push_back(x);
            vecY.push_back(y);
            vec.push_back(resizedTile);
        }
    }
    return vec;
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
        img = imgPtr->image.clone();
    }
    catch (cv_bridge::Exception& ex)
    {
        ROS_ERROR("cv_bridge exception: %s", ex.what());
        exit(-1);
    }
    
    std::vector<cv::Mat> images;
    images.push_back(img);
    if (m_isRotating)
        images.push_back(deblurring(img));
    
    aruco::MarkerDetector detector;
    std::vector<aruco::Marker> markers;
    bool addedMarkers[256] = {false};
    char frameName[100];
    int nbSubBlocks = 3;
    
    for (std::vector<cv::Mat>::iterator it = images.begin() ; it != images.end() ; it++)
    {
        for (int l=0 ; l < 3 ; l++)
        {
            cv::Mat frame = *it;
            if (l == 1)
                frame = binarizeImage(frame, false);
            else if (l == 2)
                frame = binarizeImage(frame, true);
            
            /*if (l > 0)
            {
                snprintf(frameName, 100, "Frame %d", l);
                cv::imshow(frameName, frame);
                cv::waitKey(1);
            }*/
            
            std::vector<aruco::Marker> newMarkers;
            detector.detect(frame, newMarkers);
            for (std::vector<aruco::Marker>::iterator subIt = newMarkers.begin() ; subIt != newMarkers.end() ; subIt++)
            {
                aruco::Marker& marker = *subIt;
                if (!addedMarkers[marker.id])
                {
                    addedMarkers[marker.id] = true;
                    markers.push_back(marker);
                }
            }
            
            std::vector<int> vecX, vecY;
            std::vector<cv::Mat> tiles = splitImageAndZoom(*it, nbSubBlocks, vecX, vecY);
            int n = tiles.size();
            for (int i=0 ; i < n ; i++)
            {
                for (int j=0 ; j < 3 ; j++)
                {
                    cv::Mat& tile = tiles[i];
                    if (j == 1)
                        tile = binarizeImage(tile, true);
                    else if (j == 2)
                        tile = binarizeImage(tile, false);
                    
                    detector.detect(tile, newMarkers);
                    for (std::vector<aruco::Marker>::iterator subIt = newMarkers.begin() ; subIt != newMarkers.end() ; subIt++)
                    {
                        aruco::Marker marker = *subIt;
                        if (!addedMarkers[marker.id])
                        {
                            addedMarkers[marker.id] = true;
                            for (int k=0 ; k < 4 ; k++)
                            {
                                marker[k].x /= nbSubBlocks;
                                marker[k].y /= nbSubBlocks;
                                marker[k].x += vecX[i];
                                marker[k].y += vecY[i];
                            }
                            markers.push_back(marker);
                        }
                    }
                }
            }
        }
    }

    publishAndDrawMarkers(img, markers, msg->header.stamp);
}

void DetectMarker::publishAndDrawMarkers(cv::Mat& frame, std::vector<aruco::Marker> &markers, ros::Time time)
{
    cv::Scalar colorScalar(255, 155, 0, 0);
    detect_marker::MarkersInfos markersInfos;
    markersInfos.time = time;
    
    int width = frame.cols;
    int height = frame.rows;
    int channels = frame.channels();
    
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
            
            double markerHeight = ((corners[3].y-corners[0].y) + (corners[2].y-corners[1].y)) / 2.0;
            double markerWidth = markerHeight;  //Avoids error due to perspective
            markerInfo.x = 2*(center.x /(double)width)-1;
            markerInfo.y = 2*(center.y/(double)height)-1;
            
            markerInfo.dz = MARKER_SIZE * MARKER_REF_DIST / markerHeight;
            markerInfo.dx = MARKER_SIZE * (center.x-width/2) / markerWidth;
            markerInfo.dy = MARKER_SIZE * (center.y-height/2) / markerHeight;
            markerInfo.d = sqrt(markerInfo.dy*markerInfo.dy + markerInfo.dx*markerInfo.dx + markerInfo.dz*markerInfo.dz);
            
            markerInfo.id = marker.id;
            markersInfos.infos.push_back(markerInfo);

            //ROS_INFO("Marker %d: pos = (%.3f, %.3f); d = %.3f", markerInfo.id, markerInfo.x, markerInfo.y, markerInfo.d);
        }
    }
    cv::imshow("Camera view", frame);
    cv::waitKey(1);
    
    //ROS_INFO("Publishing %lu marker infos.", markersInfos.infos.size());
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

void DetectMarker::detect()
{
    ROS_INFO("Starting detection.");
    ros::spin();
}

cv::Mat DetectMarker::deblurring(cv::Mat img)
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
    for(i=0 ; i<10 ; i++)
    {
        if(i > 1)
        {
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
    
    J2.convertTo(J2, CV_8U, 255.0);
    return J2;
}

cv::Mat DetectMarker::binarizeImage(cv::Mat& img, bool strong)
{
    cv::Mat grayImg, binImg;
    
    if (img.channels() > 1)
        cv::cvtColor(img, grayImg, CV_BGR2GRAY);
    else
        grayImg = img.clone();
    
    cv::threshold(grayImg, binImg, strong ? 50 : 178, 255, cv::THRESH_BINARY);
    return binImg;
}



