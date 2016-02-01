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
            std::vector<Rectangle> rects = getImageRects(img);
            return matchPerspective(img, rects, *it);
        }
    }
    
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
    
    cv::rectangle(colorImg, result.boundingRect.ul , result.boundingRect.lr, cv::Scalar(0, 255, 0), 2);
    
    cv::Mat fullDisplay, colorDiffImg = result.colorDiffImg.clone();
    if (colorDiffImg.rows < colorImg.rows)
        copyMakeBorder(colorDiffImg, colorDiffImg, 0, colorImg.rows-colorDiffImg.rows, 0, 0, cv::BORDER_CONSTANT);
    else if (colorImg.rows < colorDiffImg.rows)
        copyMakeBorder(colorImg, colorImg, 0, colorDiffImg.rows-colorImg.rows, 0, 0, cv::BORDER_CONSTANT);
    
    cv::hconcat(colorImg, colorDiffImg, fullDisplay);
    return fullDisplay;
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

cv::Mat FriendMatcher::binarizeImageKMeans(const cv::Mat& img)
{
    cv::Mat points = img.clone().reshape(0, img.cols*img.rows);
    points.convertTo(points, CV_32F, 1/255.0);
    cv::Mat labels;
    cv::kmeans(points, 2, labels, cv::TermCriteria( cv::TermCriteria::EPS+cv::TermCriteria::COUNT, 10, 1.0), 6, cv::KMEANS_PP_CENTERS);
    if (cv::sum(labels)[0] > img.cols*img.rows/2)
        labels = 1 - labels;
    labels.convertTo(labels, CV_8U, 255);
    return labels.reshape(0, img.rows);
}

cv::Mat FriendMatcher::computeDistanceImage(const cv::Mat& img, cv::Scalar color)
{
    cv::Mat imgCopy = img.clone();    
    imgCopy.convertTo(imgCopy, CV_32F, 1/255.0);
    
    cv::Mat matColor = cv::Mat::zeros(1, 1, CV_32FC3);
    matColor.at<cv::Vec3f>(0,0) = cv::Vec3f(color[0]/255.0, color[1]/255.0, color[2]/255.0);
    cv::cvtColor(matColor, matColor, CV_RGB2Lab);
    cv::Vec3f colorVec = matColor.at<cv::Vec3f>(0,0);
    color = cv::Scalar(colorVec[0], colorVec[1], colorVec[2]);
    
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

std::vector<FriendMatcher::Rectangle> FriendMatcher::getImageRects(const cv::Mat& img) const
{
    std::vector<Rectangle> vec;
    Rectangle rect;
    
    //test7.png
    /*rect.ul.x = 108;
    rect.ul.y = 215;
    rect.lr.x = 134;
    rect.lr.y = 291;*/
    
    //test4.png
    /*rect.ul.x = 292;
    rect.ul.y = 203;
    rect.lr.x = 377;
    rect.lr.y = 295;*/
    
    //test3.png
    rect.ul.x = 350;
    rect.ul.y = 209;
    rect.lr.x = 396;
    rect.lr.y = 254;
    
    vec.push_back(rect);
    return vec;
}

FriendMatcher::MatchResult FriendMatcher::matchPerspective(const cv::Mat& aFullImg, const std::vector<FriendMatcher::Rectangle>& rects, const FriendMatcher::TemplateInfo& templ) const
{
    MatchResult bestResult;
    bestResult.score = -1;
    
    for (std::vector<Rectangle>::const_iterator it = rects.begin() ; it != rects.end() ; it++)
    {
        Rectangle lRectImg = *it;
        Rectangle lRectTemplate = templ.roi;
        int iw = aFullImg.cols, ih = aFullImg.rows;
        int tw = templ.image.cols, th = templ.image.rows;
        
        ROS_INFO("lRectImg = (%d, %d) (%d, %d)", lRectImg.ul.x, lRectImg.ul.y, lRectImg.lr.x, lRectImg.lr.y);
        ROS_INFO("lRectTemplate = (%d, %d) (%d, %d)", lRectTemplate.ul.x, lRectTemplate.ul.y, lRectTemplate.lr.x, lRectTemplate.lr.y);
        
        double fScaleX = (lRectImg.lr.x - lRectImg.ul.x) / (double)(lRectTemplate.lr.x - lRectTemplate.ul.x);
        double fScaleY = (lRectImg.lr.y - lRectImg.ul.y) / (double)(lRectTemplate.lr.y - lRectTemplate.ul.y);
        ROS_INFO("fScaleX = %.3f, fScaleY = %.3f", fScaleX, fScaleY);
        
        double fDistanceZ = templ.h * MARKER_REF_DIST / (lRectImg.lr.y - lRectImg.ul.y);
        double fMinBackScale = fDistanceZ / (fDistanceZ + templ.h);
        double fRotY = acos(std::min(1.0, fScaleX / fScaleY));
        ROS_INFO("fDistanceZ = %.3f, fMinBackScale = %.3f, fRotY = %.3f", fDistanceZ, fMinBackScale, fRotY*180/M_PI);
        
        Rectangle lCropRect;
        lCropRect.ul.x = std::max(0, lRectImg.ul.x - (int)ceil(lRectTemplate.ul.x * fScaleX));
        lCropRect.ul.y = std::max(0, lRectImg.ul.y - (int)ceil(lRectTemplate.ul.y * fScaleY));
        lCropRect.lr.x = std::min(iw, lRectImg.lr.x + (int)ceil((tw-lRectTemplate.lr.x) * fScaleX));
        lCropRect.lr.y = std::min(ih, lRectImg.lr.y + (int)ceil((th-lRectTemplate.lr.y) * fScaleY));
        ROS_INFO("lCropRect = (%d, %d) (%d, %d)", lCropRect.ul.x, lCropRect.ul.y, lCropRect.lr.x, lCropRect.lr.y);
        
        cv::Mat aImg = aFullImg(cv::Rect(lCropRect.ul.x, lCropRect.ul.y, lCropRect.lr.x-lCropRect.ul.x, lCropRect.lr.y-lCropRect.ul.y));
        iw = aImg.cols;
        ih = aImg.rows;
        
        cv::Mat aTemplate;
        ROS_INFO("w = %d ; h = %d", iw, ih);
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
        
        double fBestDiff = std::numeric_limits<double>::infinity();
        const double fAngleMargin = 10 * M_PI / 180;
        const double fAngleStep = 5 * M_PI / 180;
        for (double fAngleY = fRotY - fAngleMargin ; fAngleY <= fRotY + fAngleMargin ; fAngleY += fAngleStep)
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
