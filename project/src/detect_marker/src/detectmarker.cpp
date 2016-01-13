#include "detectmarker.h"

DetectMarker::DetectMarker()
{
    pub = nh.advertise<detect_marker::Markers>("/int8_marker_ids", 10);
    sub =  nh.subscribe("/camera/rgb/image_raw", 1, &DetectMarker::rgbCallback, this);
    vectorSub = nh.subscribe("/ar_multi_boards/position",1, &DetectMarker::vectorDetectCallback, this);
    nextId = 0;
}

void DetectMarker::vectorDetectCallback(const geometry_msgs::Vector3StampedPtr &msg)
{
    cout << "[" << msg->vector.x << "," << msg->vector.y << "," << msg->vector.z << "]" << endl;
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

    cv::Scalar colorScalar(255,155,0, 0);
    cv::Mat frame = cv_ptr->image;

    for (int i=0; i<8; i++)
         ui8_markers.marker[i] = 8;

    detector.detect(cv_ptr->image,detMarker);


    ROS_INFO("Amount of markers: %ld", detMarker.size());

    for (int i=0; i<detMarker.size(); i++)
    {
        //if (detMarker[i].id == nextId)
        {
            DetectMarker::publishMarker();
            detMarker[i].draw(frame, colorScalar);

            //detMarker[i].calculateExtrinsics(detMarker[i].getArea(),frame);

            aruco::MarkerInfo(detMarker[i].id);



            nextId++;
        }
    }

    //marker.draw(cv_ptr->image, cv::Scalar(190, 10, 111),1, true);
    //ROS_INFO("Area %f", marker.getArea());
    cv::imshow("Marker Detection",frame);
    cv::waitKey(30); //!!!!!!
}
void DetectMarker::Detection()
{
    ros::Rate   rate(100);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
}
