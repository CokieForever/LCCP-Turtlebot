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


/**
*@class DetectFriend
*@brief This class is used to detect the three different friends(star, mushroom, coin). The detection is done by recognizing their colors and  returning the coordinates of a rectangular around them
*/  
class DetectFriend
{
    public:
        DetectFriend(ros::NodeHandle& nodeHandle); ///<Constructor
	void Identification(); ///<function called to start identification
        const static double min_score =0.85; ///<after friend matching a score is calculated. min_score is the bound of acceptable items.
        static const double FRIEND_REF_DIST = 480 * 0.2 / 0.175; ///< distance of robot, according to which the whole image of a friend is in the image taken by the camera
        static const double COIN_HEIGHT = 0.1785; ///<real height of the coin image
	static const double MUSHROOM_HEIGHT=0.1785; ///< real height of the mushroom image
	static const double STAR_HEIGHT=0.1865; ///< real height of the star image
    private:
        ros::NodeHandle& m_nodeHandle; ///< Main node handle
        detect_friend::Friend_id publish_infos_of_friend(const cv::Mat& img, int i, cv::Rect rectangle); ///< pusblish the infos of the detected friend (id, center of the friend, distance of friend center from recorded image center, time)
	static bool ComputeLinesIntersection(cv::Point linePoints1[2], cv::Point linePoints2[2], cv::Point *isectPoint);///<used to compute the center of the friend in the recorded image
        static bool ComputeQuadrilateralCenter(cv::Point points[4], cv::Point *centerPoint);///<used to compute the center of the friend in the recorded image
        ros::Subscriber m_cameraSub; ///<subscriber to camera "/camera/rgb/image_raw"
        ros::Publisher m_friend_idPub; ///< publisher of the friend information "/friendinfo"
        cv::Mat star_image; ///<reference image of star
        cv::Mat mushroom_image; ///<reference image of mushroom
        cv::Mat coin_image; ///<reference image of coin
        FriendMatcher m_friendmatcher; ///< object of FriendMatcher class
        void cameraSubCallback(const sensor_msgs::Image::ConstPtr& msg); 
        
};
