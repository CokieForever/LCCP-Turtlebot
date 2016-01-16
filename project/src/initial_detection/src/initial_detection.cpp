#include "geometry_msgs/Twist.h"
#include "initial_detection.h"
#include "sensor_msgs/LaserScan.h"

//Constructor
initial_detection::initial_detection()
{
    m_next_id_detected = false;
    m_angularSpeed=0;
    m_linearSpeed=0;
    m_obstacle = false;
    m_next_id = 0;
    m_commandPub = m_node.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);
    m_markerSub = m_node.subscribe("/markerinfo", 10, &initial_detection::detectCallback, this);
    m_scanSub = m_node.subscribe ("/scan",1, &initial_detection::scanCallback, this);

    int nbRanges = ceil(360 / ANGLE_PRECISION);
	m_ranges = new double[nbRanges];
	for (int i=0 ; i < nbRanges ; i++)
		m_ranges[i] = nan("");
}

initial_detection::~initial_detection()
{
	if (m_ranges != NULL)
		delete m_ranges;
}

void initial_detection::startRotating()
{
    ros::Rate rate(10);
    ROS_INFO("Start Rotation");
    bool initialization = true;

    while(ros::ok())
    {
        if ( (m_obstacle = proximityAlert()) )
        {
	        ROS_INFO("Obstacle!");
            if (!initialization)
             {   m_next_id_detected=false;
                return;
             }
        }

        if (initialization && m_linearSpeed > 0)
        {
            initialization = false;
        }

        geometry_msgs::Twist msg;
        msg.angular.z = m_angularSpeed;
        msg.linear.x= m_obstacle ? 0 : m_linearSpeed;
        ROS_INFO("iT SHOULD ROTATE");
        m_commandPub.publish(msg);
        ros::spinOnce();
        rate.sleep();
    }

}



//Callback of the /markerinfo topic. Checks the information of the Markers that are detected
void initial_detection::detectCallback(const detect_marker::MarkersInfos::ConstPtr& markersInfos)
{
    int nbInfos = markersInfos->infos.size();
    double xcenter, ycenter;

    for (int i=0 ; i < nbInfos ; i++)
    {
        const detect_marker::MarkerInfo& detected_Marker = markersInfos->infos[i];

        int id = detected_Marker.id;
        xcenter= detected_Marker.x;
        ycenter= detected_Marker.y;
		
        if (m_next_id==id && m_next_id_detected==false) 
        { 
        	m_next_id = id+1;
	        m_next_id_detected=true;
            break;
        }
    }
    
    if (m_next_id_detected==true && fabs(xcenter)<0.1)
    {
        m_angularSpeed = 0;
        m_linearSpeed = 0.2;
        ROS_INFO("iT SHOULD HAVE FOUND THE CENTER");
    }
    else if(m_next_id_detected==true && fabs(xcenter)>=0.1) 
    {
        m_angularSpeed = -xcenter/3;
        m_linearSpeed = 0.2;
        ROS_INFO("iT SHOULD HAVE FOUND THE ID NOT THE CENTER");
    }
    else
    {
        m_angularSpeed = 0.33;
        m_linearSpeed = 0;
    }
}


void initial_detection::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int nbRanges = ceil((scan->angle_max - scan->angle_min) / scan->angle_increment);
	int prevAngleIdx = -1;
	for (int i=0 ; i < nbRanges ; i++)
	{
		double angle = modAngle(-(scan->angle_min + i * scan->angle_increment));
		int angleIdx = floor(angle * 180 / (M_PI * ANGLE_PRECISION));
		if (angleIdx != prevAngleIdx)
		{
			prevAngleIdx = angleIdx;
			m_ranges[angleIdx] = scan->ranges[i];
		}
		else if (m_ranges[angleIdx] > scan->ranges[i])
			m_ranges[angleIdx] = scan->ranges[i];
	}
}

bool initial_detection::proximityAlert()
{
	int idx1 = ceil(145 / ANGLE_PRECISION);
	int idx2 = ceil(215 / ANGLE_PRECISION);
	for (int i=idx1 ; i < idx2 ; i++)
	{
		if (m_ranges[i] <= MIN_PROXIMITY_RANGE_M)
			return true;
	}
	return false;
}

double initial_detection::modAngle(double rad)
{
	return fmod(fmod(rad, 2*M_PI) + 2*M_PI, 2*M_PI);
}


