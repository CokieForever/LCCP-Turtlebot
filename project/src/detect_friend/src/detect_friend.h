//#ifndef DETECTFRIEND_H
//#define DETECTFRIEND_H
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "cv_bridge/cv_bridge.h"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/imgproc/imgproc.hpp>

class DetectFriend
{
    public:
        DetectFriend(ros::NodeHandle& nodeHandle);
	void Identification();
        const static double min_score =0.7;
    private:
        ros::NodeHandle& m_nodeHandle;
        ros::Subscriber m_cameraSub;
        ros::Publisher m_friend_idPub;
        cv::Mat star_image;
        cv::Mat mushroom_image;
        cv::Mat coin_image;
        void cameraSubCallback(const sensor_msgs::Image::ConstPtr& msg);
        
};
