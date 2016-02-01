#include <ros/ros.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "friendmatcher.h"

int main( int argc, char** argv )
{
    ros::init(argc, argv, "orb_test");
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

    FriendMatcher matcher;    
    FriendMatcher::TemplateInfo templateInfo;
    templateInfo.image = img_object;
    templateInfo.roi.ul.x = 29;
    templateInfo.roi.ul.y = 28;
    templateInfo.roi.lr.x = 29+354;
    templateInfo.roi.lr.y = 28+379;
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
    
    ROS_INFO("Corner 1: %d, %d", result.boundingRect.ul.x, result.boundingRect.ul.y);
    ROS_INFO("Corner 2: %d, %d", result.boundingRect.lr.x, result.boundingRect.lr.y);
    ROS_INFO("Score: %.2f%%", result.score*100);
    
    cv::Mat display = matcher.drawResult(img_scene, result);
    cv::imshow("Result", display);
    cv::waitKey(0);
    
    return 0;
}
        