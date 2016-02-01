#include <ros/ros.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "friendmatcher.h"

int main( int argc, char** argv )
{
    ros::init(argc, argv, "friendmatcher_test");
    ros::NodeHandle node("~");
    ROS_INFO("Initialized ROS.");
    
    std::string templatePath = "";
    if (!node.getParam("template_path", templatePath))
    {
        ROS_ERROR("The template path was not provided.");
        return 1;
    }
    
    std::string imagePath = "";
    if (!node.getParam("image_path", imagePath))
    {
        ROS_ERROR("The image path was not provided.");
        return 1;
    }
    
    cv::Mat img_object = cv::imread( templatePath );
    cv::Mat img_scene = cv::imread( imagePath );

    if( !img_object.data || !img_scene.data )
    {
        ROS_ERROR("Error while reading images (%s, %s)", imagePath.c_str(), templatePath.c_str());
        return 1;
    }
    
    cv::Mat distImg = FriendMatcher::computeDistanceImage(img_scene, cv::Scalar(255,235,0));
    cv:threshold(distImg, distImg, 0.1, 1.0, cv::THRESH_BINARY_INV);
    cv::imshow("Lab Space", distImg);
    cv::waitKey(0);

    FriendMatcher matcher;    
    FriendMatcher::TemplateInfo templateInfo;
    templateInfo.image = img_object;
    templateInfo.roi.x = 29;
    templateInfo.roi.y = 28;
    templateInfo.roi.width = 354;
    templateInfo.roi.height = 379;
    templateInfo.mainColor = cv::Scalar(255, 237, 0);
    templateInfo.h = 0.09;
    templateInfo.w = 0.09;
    templateInfo.name = "star";
    templateInfo.id = 0;
    
    matcher.addTemplate(templateInfo);
    FriendMatcher::MatchResult result = matcher.match(img_scene, templateInfo.id);
    
    if (result.score < 0)
    {
        ROS_INFO("No match found.");
        return 0;
    }
    
    ROS_INFO("Corner 1: %d, %d", result.boundingRect.x, result.boundingRect.y);
    ROS_INFO("Corner 2: %d, %d", result.boundingRect.br().x, result.boundingRect.br().y);
    ROS_INFO("Score: %.2f%%", result.score*100);
    
    cv::Mat display = matcher.drawResult(img_scene, result);
    cv::imshow("Result", display);
    cv::waitKey(0);
    
    return 0;
}
        
