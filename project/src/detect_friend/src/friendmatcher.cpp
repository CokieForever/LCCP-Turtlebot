#include <ros/ros.h>
#include "friendmatcher.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

FriendMatcher::FriendMatcher(const std::vector<FriendMatcher::TemplateInfo>& templates): m_templates(std::vector<TemplateInfo>())
{
    for (std::vector<TemplateInfo>::const_iterator it = templates.begin() ; it != templates.end() ; it++)
        addTemplate(*it);
}

bool FriendMatcher::addTemplate(const TemplateInfo& templ)
{
    //TODO avoid inserting twice the same ID
    TemplateInfo templCopy = templ;
    
    /*std::vector<Rectangle> rects = getImageRects(templ.image);
    if (rects.size() <= 0)
        return false;
    
    double maxArea = 0;
    for (std::vector<Rectangle>::iterator it = rects.begin() ; it != rects.end() ; it++)
    {
        double area = (it->lr.x - it->ul.x) * (it->lr.y - it->ul.y);
        if (area > maxArea)
        {
            maxArea = area;
            templCopy.roi = *it;
        }
    }*/
    
    templCopy.image = templ.image.clone();
    m_templates.push_back(templCopy);
    return true;
}

FriendMatcher::MatchResult FriendMatcher::match(const cv::Mat& img, int templateId) const
{
    for (std::vector<TemplateInfo>::const_iterator it = m_templates.begin() ; it != m_templates.end() ; it++)
    {
        if (it->id == templateId)
        {
            std::vector<cv::Rect> rects = getImageRects(img, it->mainColor);
            //ROS_INFO("Number of rects: %lu", rects.size());
            return matchPerspective(img, rects, *it);
        }
    }

    ROS_WARN("Template #%d not found.", templateId);
    
    MatchResult result;
    result.score = -1;
    return result;
}

cv::Mat FriendMatcher::drawResult(const cv::Mat& img, const MatchResult& result) const
{
    cv::Mat colorImg;
    if (img.channels() < 3)
        cvtColor(img, colorImg, CV_GRAY2RGB);
    else
        colorImg = img.clone();
    
    cv::rectangle(colorImg, result.boundingRect.tl() , result.boundingRect.br(), cv::Scalar(0, 255, 0), 2);
    
    if (result.score >= 0)
    {
        cv::Mat fullDisplay, colorDiffImg = result.colorDiffImg.clone();
        if (colorDiffImg.rows < colorImg.rows)
            copyMakeBorder(colorDiffImg, colorDiffImg, 0, colorImg.rows-colorDiffImg.rows, 0, 0, cv::BORDER_CONSTANT);
        else if (colorImg.rows < colorDiffImg.rows)
            copyMakeBorder(colorImg, colorImg, 0, colorDiffImg.rows-colorImg.rows, 0, 0, cv::BORDER_CONSTANT);
        
        //ROS_INFO("h1 = %d, h2 = %d", colorImg.rows, colorDiffImg.rows);
        //ROS_INFO("type1 = %d, type2 = %d", colorImg.type(), colorDiffImg.type());
        
        cv::hconcat(colorImg, colorDiffImg, fullDisplay);
        return fullDisplay;
    }
    else
        return colorImg;
}

cv::Mat FriendMatcher::applyLogFilter(const cv::Mat& img)
{
    static double logKernel[5][5] = {{0.0448, 0.0468,  0.0564, 0.0468, 0.0448},
                                     {0.0468, 0.3167,  0.7146, 0.3167, 0.0468},
                                     {0.0564, 0.7146, -4.9078, 0.7146, 0.0564},
                                     {0.0468, 0.3167,  0.7146, 0.3167, 0.0468},
                                     {0.0448, 0.0468,  0.0564, 0.0468, 0.0448}};
    cv::Mat kernelMat = cv::Mat(5, 5, CV_64F, logKernel);

    cv::Mat filteredImg;
    cv::filter2D(img, filteredImg, -1, kernelMat);
    return filteredImg;
}

cv::Mat FriendMatcher::binarizeImage(const cv::Mat& img, double threshold, double maxVal)
{
    cv::Mat binImg;
    cv::threshold(img, binImg, threshold, maxVal, cv::THRESH_BINARY);
    return binImg;
}


cv::Mat FriendMatcher::binarizeImageKMeans(const cv::Mat& img)
{
    cv::Mat points = img.clone().reshape(0, img.cols*img.rows);
    points.convertTo(points, CV_32F, 1/255.0);
    cv::Mat labels;
    cv::kmeans(points, 2, labels, cv::TermCriteria( cv::TermCriteria::EPS+cv::TermCriteria::COUNT, 1000, 0.01), 1, cv::KMEANS_PP_CENTERS);
    if (cv::sum(labels)[0] > img.cols*img.rows/2)
        labels = 1 - labels;
    labels.convertTo(labels, CV_8U, 255);
    return labels.reshape(0, img.rows);
}

cv::Scalar FriendMatcher::convertToLabSpace(const cv::Scalar& color)
{
    cv::Mat matColor = cv::Mat::zeros(1, 1, CV_32FC3);
    matColor.at<cv::Vec3f>(0,0) = cv::Vec3f(color[0]/255.0, color[1]/255.0, color[2]/255.0);
    cv::cvtColor(matColor, matColor, CV_RGB2Lab);
    cv::Vec3f colorVec = matColor.at<cv::Vec3f>(0,0);
    return cv::Scalar(colorVec[0], colorVec[1], colorVec[2]);
}

cv::Mat FriendMatcher::computeDistanceImage(const cv::Mat& img, cv::Scalar color)
{
    cv::Mat imgCopy = img.clone();    
    imgCopy.convertTo(imgCopy, CV_32F, 1/255.0);
    
    color = convertToLabSpace(color);
    
    cv::cvtColor(imgCopy, imgCopy, CV_BGR2Lab);
    
    cv::Mat diff = imgCopy - color;
    cv::pow(diff, 2, diff);
    diff = diff.reshape(1, imgCopy.rows*imgCopy.cols);
    
    cv::Mat distMat;
    cv::reduce(diff, distMat, 1, CV_REDUCE_SUM);
    distMat /= 139032;
    cv::sqrt(distMat, distMat);
    distMat = distMat.reshape(1, imgCopy.rows);
    
    return distMat;
}

cv::Mat FriendMatcher::removeIsolatedPixels(const cv::Mat& binImg, int nbMinNeighbours)
{
    cv::Mat binImgCopy = binImg.clone();
    binImgCopy.convertTo(binImgCopy, CV_32F, 1/255.0);
    float kernel[3][3] = {{1, 1, 1},
                          {1, 0, 1},
                          {1, 1, 1}};
    cv::Mat kernelMat = cv::Mat(3, 3, CV_32F, kernel);
    cv::Mat filtered, output;
    cv::filter2D(binImgCopy, filtered, -1, kernelMat);
    cv::threshold(filtered, output, nbMinNeighbours-0.5, 1.0, cv::THRESH_BINARY);
    output = output.mul(binImgCopy);
    output.convertTo(output, CV_8U, 255);
    
    return output;
}

std::vector<cv::Rect> FriendMatcher::getImageRects(const cv::Mat& img, cv::Scalar scalarcolor)
{
    cv::Scalar scalarcolor2 = convertToLabSpace(scalarcolor);
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    double area;
    cv::Mat image_object, image_object2;
    cv::Mat image_scene;
    cv::Mat image_scene2;
    std::vector<cv::Mat> rgb1;
    cv::Mat add_res, add_res2, euclidean1, euclidean2;
    cv::Mat imgCopysc=img.clone();
    imgCopysc.convertTo(imgCopysc, CV_32FC3, 1/255.0);
    cv::cvtColor(imgCopysc, imgCopysc, CV_BGR2Lab);

    //Calculate the absolute difference between the scalar and the image
    cv::absdiff(imgCopysc, scalarcolor2, image_scene);

    //Calculate the pow
    cv::pow(image_scene, 2, image_scene2);

    //Split the channels in order to calculate the sum
    cv::split(image_scene2, rgb1);
    add_res=rgb1[0]+rgb1[1]+rgb1[2];
    //sgrt of the sum. Final DIstance
    cv::sqrt(add_res, euclidean1);
    //normalization according to maximum distance
    euclidean1=(euclidean1/372.87);

    cv::Mat scene, obj;

    //Set the pixels with euclidean distance from the scalar <0.15

    cv::threshold(euclidean1, scene, 0.20, 1, cv::THRESH_BINARY_INV);
    /*cv::imshow("Scene", scene);
    cv::waitKey(1);*/

    //find contours in the image.
    cv::Mat euclideanCopy2=scene.clone();
    euclideanCopy2.convertTo(euclideanCopy2, CV_8U, 255);
    cv::Mat aFullImgCopy = euclideanCopy2.clone();

    cv::findContours( euclideanCopy2, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

    //ignore the black contours
    for(int i=0; i<contours.size(); i++)
    {
        if (cv::contourArea(contours[i],true) > 0)
        {
            //ROS_INFO("black");
            contours.erase(contours.begin() + i);
            hierarchy.erase(hierarchy.begin()+i);
            i--;
        }
        else
        {
            //ROS_INFO("white");
        }
    }

    //ROS_INFO("contour size %lu", contours.size() );
    std::vector<std::vector<cv::Point> > contours_poly( contours.size() );
    std::vector<cv::Rect> boundRect;

    //create rectangles around contours of interest and draw
    for( int i = 0; i < contours.size(); i++ )
    {
        cv::approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 3, true );
        area=cv::contourArea(contours_poly[i]);
        //ROS_INFO("area %.3f", area );
        //ignore the contours that have area less than 10*10
        if (area<100)
        {
            contours.erase(contours.begin() + i);
            hierarchy.erase(hierarchy.begin()+i);
            i--;
        }
        else
        {
            cv::Rect rect = cv::boundingRect( cv::Mat(contours_poly[i]) );
            boundRect.push_back(rect);

            cv::rectangle(aFullImgCopy, rect.tl(), rect.br(), cv::Scalar(255,255,255), 2);
        }
    }

    /*cv::imshow("Rects", aFullImgCopy);
    cv::waitKey(1);*/

    return boundRect;
}

FriendMatcher::MatchResult FriendMatcher::compareBinaryImages(const cv::Mat& binImg1, const cv::Mat& binImg2)
{
    cv::Mat binImg1Copy = removeIsolatedPixels(binImg1, 2);
    cv::Mat binImg2Copy = removeIsolatedPixels(binImg2, 2);
    
    cv::Mat binFDImg, binFDTemplate;
    int dilatation = std::max(4, (int)round(DILATATION_FACTOR * binImg2.rows));
    cv::Mat kernelMat = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(dilatation, dilatation));
    cv::dilate(binImg1Copy, binFDImg, kernelMat);
    cv::dilate(binImg2Copy, binFDTemplate, kernelMat);
    
    cv::Mat binDiffImg1 = binImg2Copy > binFDImg;
    cv::Mat binDiffImg2 = binImg1Copy > binFDTemplate;
    
    double ref = cv::sum(binImg2Copy)[0] / (255.0*binImg2Copy.rows*binImg2Copy.cols);
    double diff1 = cv::sum(binDiffImg1)[0] / (255.0*binDiffImg1.rows*binDiffImg1.cols);
    double diff2 = cv::sum(binDiffImg2)[0] / (255.0*binDiffImg2.rows*binDiffImg2.cols);
    
    MatchResult result;
    result.score = 1.0 - std::min(1.0, std::max(diff1/ref, diff2/ref));
    cv::Mat planes[3] = {binDiffImg1, binImg2Copy, binDiffImg2};
    cv::Mat colorDiffImg;
    cv::merge(planes, 3, result.colorDiffImg);
    
    return result;
}

FriendMatcher::MatchResult FriendMatcher::matchPerspective(const cv::Mat& aFullImg, const std::vector<cv::Rect>& rects, const FriendMatcher::TemplateInfo& templ) const
{
    MatchResult bestResult;
    bestResult.score = -1;
    
    for (std::vector<cv::Rect>::const_iterator it = rects.begin() ; it != rects.end() ; it++)
    {
        cv::Rect lRectImg = *it;
        cv::Rect lRectTemplate = templ.roi;
        int iw = aFullImg.cols, ih = aFullImg.rows;
        int tw = templ.image.cols, th = templ.image.rows;
        //ROS_INFO("iw = %d, ih = %d", iw, ih);
        //ROS_INFO("tw = %d, th = %d", tw, th);

        //ROS_INFO("lRectImg = (%d, %d) (%d, %d)", lRectImg.tl().x, lRectImg.tl().y, lRectImg.br().x, lRectImg.br().y);
        //ROS_INFO("lRectTemplate = (%d, %d) (%d, %d)", lRectTemplate.tl().x, lRectTemplate.tl().y, lRectTemplate.br().x, lRectTemplate.br().y);
        
        double fScaleX = (lRectImg.br().x - lRectImg.tl().x) / (double)(lRectTemplate.br().x - lRectTemplate.tl().x);
        double fScaleY = (lRectImg.br().y - lRectImg.tl().y) / (double)(lRectTemplate.br().y - lRectTemplate.tl().y);
        //ROS_INFO("fScaleX = %.3f, fScaleY = %.3f", fScaleX, fScaleY);
        
        double fDistanceZ = templ.h * MARKER_REF_DIST / (lRectImg.br().y - lRectImg.tl().y);
        double fMinBackScale = fDistanceZ / (fDistanceZ + templ.h);
        double fRotY = acos(std::min(1.0, fScaleX / fScaleY));
        //ROS_INFO("fDistanceZ = %.3f, fMinBackScale = %.3f, fRotY = %.3f", fDistanceZ, fMinBackScale, fRotY*180/M_PI);
        
        cv::Rect lCropRect;
        lCropRect.x = std::max(0, lRectImg.tl().x - (int)ceil(lRectTemplate.tl().x * fScaleX));
        lCropRect.y = std::max(0, lRectImg.tl().y - (int)ceil(lRectTemplate.tl().y * fScaleY));
        lCropRect.width = std::min(iw - lCropRect.x, (int)round(tw * fScaleX));
        lCropRect.height = std::min(ih - lCropRect.y, (int)round(th * fScaleY));
        //ROS_INFO("lCropRect = (%d, %d) (%d, %d)", lCropRect.tl().x, lCropRect.tl().y, lCropRect.br().x, lCropRect.br().y);
        
        cv::Mat aImg = aFullImg(cv::Rect(lCropRect.tl().x, lCropRect.tl().y, lCropRect.br().x-lCropRect.tl().x, lCropRect.br().y-lCropRect.tl().y));
        iw = aImg.cols;
        ih = aImg.rows;
        
        cv::Mat aTemplate;
        //ROS_INFO("w = %d ; h = %d", iw, ih);
        cv::resize(templ.image, aTemplate, cv::Size(iw,ih), 0, 0, cv::INTER_NEAREST);
        tw = aTemplate.cols;
        th = aTemplate.rows;
        
        const int ptsX[4] = {0, tw, 0, tw}, ptsY[4] = {0, 0, th, th};
        cv::Point2f lPoints1[4], lPoints2[4];
        for (int i=0 ; i < 4 ; i++)
        {
            lPoints1[i].x = ptsX[i]; lPoints2[i].x = ptsX[i];
            lPoints1[i].y = ptsY[i]; lPoints2[i].y = ptsY[i];
        }
        
        cv::Mat aBinImg = binarizeImageKMeans(applyLogFilter(aImg));
        /*cv::imshow("bin", aBinImg);
        cv::waitKey(1);*/
        
        double fBestDiff = std::numeric_limits<double>::infinity();
        const double fAngleMargin = 10 * M_PI / 180;
        const double fAngleStep = 5 * M_PI / 180;
        for (double fAngleY = fRotY - fAngleMargin ; fAngleY <= fRotY + fAngleMargin && fabs(fAngleY) <= 20 * M_PI / 180 ; fAngleY += fAngleStep)
        {
            double x = cos(fAngleY);
            double fBackScale = x * (1-fMinBackScale) + fMinBackScale;
            for (double y = 0 ; y <= 1-fBackScale ; y += 0.01)
            {
                lPoints2[1].y = y * th;
                lPoints2[3].y = (y + fBackScale) * th;
                cv::Mat aWarped;
                cv::warpPerspective(aTemplate, aWarped, cv::getPerspectiveTransform(lPoints1, lPoints2), cv::Size(iw,ih), cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(255,255,255));
                cv::Mat aBinTemplateResized = binarizeImageKMeans(applyLogFilter(aWarped));
                
                /*cv::imshow("Bin img", aBinImg);
                cv::imshow("Bin template", aBinTemplateResized);
                cv::waitKey(0);*/
                
                MatchResult result = compareBinaryImages(aBinImg, aBinTemplateResized);
                if (result.score > bestResult.score)
                {
                    bestResult = result;
                    bestResult.boundingRect = lCropRect;
                }
            }
        }
    }
    
    return bestResult;
}
