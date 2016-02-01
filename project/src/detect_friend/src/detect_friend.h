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
#include "friendmatcher.h"
#include "detect_friend/Friend_id.h"
#include "detect_friend/FriendsInfos.h"

class DetectFriend
{
    public:
        DetectFriend(ros::NodeHandle& nodeHandle);
	void Identification();
        const static double min_score =0.7;
        static const double FRIEND_REF_DIST = 480 * 0.2 / 0.175;
        static const double COIN_HEIGHT = 0.1785;
	static const double MUSHROOM_HEIGHT=0.1785;
	static const double STAR_HEIGHT=0.1865;
    private:
        ros::NodeHandle& m_nodeHandle;
        detect_friend::Friend_id publish_infos_of_friend(const cv::Mat& img, int i, cv::Rect rectangle);
	static bool ComputeLinesIntersection(cv::Point linePoints1[2], cv::Point linePoints2[2], cv::Point *isectPoint);
        static bool ComputeQuadrilateralCenter(cv::Point points[4], cv::Point *centerPoint);
        ros::Subscriber m_cameraSub;
        ros::Publisher m_friend_idPub;
        cv::Mat star_image;
        cv::Mat mushroom_image;
        cv::Mat coin_image;
        FriendMatcher m_friendmatcher;
        void cameraSubCallback(const sensor_msgs::Image::ConstPtr& msg);
        
};
