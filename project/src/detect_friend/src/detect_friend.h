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
using namespace cv;
class DetectFriend
{
    public:
        DetectFriend(ros::NodeHandle& nodeHandle);
        void Identification();
        struct ORBMatchResult
        {
            cv::Point2f corners[4];
            double score;
            cv::Mat colorDiffImg;
            
        };
        ORBMatchResult match(const cv::Mat& img, const cv::Mat& templ) const;
        cv::Mat drawResult(const cv::Mat& img, const ORBMatchResult& result) const;
    private:
        ros::NodeHandle& m_nodeHandle;
        ros::Subscriber m_cameraSub;
        ros::Publisher m_friend_idPub;
        Mat star_image;
        Mat mushroom_image;
        Mat coin_image;
        static cv::Mat binarizeImageKMeans(const cv::Mat& input);
        static cv::Mat removeIsolatedPixels(const cv::Mat& binInput, int nbMinNeighbours=1);
        void cameraSubCallback(const sensor_msgs::Image::ConstPtr& msg);
        void friends_inscene(const Mat &img_scene, const Mat &img_object, int j);
        
};
