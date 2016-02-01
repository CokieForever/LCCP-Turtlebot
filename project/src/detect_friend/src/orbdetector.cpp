#include "orbdetector.h"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <ros/ros.h>

using namespace cv;

static bool matchCompareFn(const DMatch& m1, const DMatch& m2);

ORBDetector::ORBDetector()
{
    
}

ORBDetector::ORBMatchResult ORBDetector::multimatchsplit(Mat& img, const Mat& templ) const
{
    ORBMatchResult bestResult;
    bestResult.score = -1;
    
    std::vector<Mat> tiles = splitImageAndZoom(img, 2);
    //tiles.push_back(img);
    for (std::vector<Mat>::iterator it=tiles.begin() ; it != tiles.end()-1 ; it++)
    {
        Mat tile = *it;
        tile = sharpen(tile);
        /*imshow("tile", tile);
        waitKey(0);*/
        
        ORBMatchResult result = multimatch(tile, templ);
        if (result.score >= 0 && result.score > bestResult.score)
        {
            img = tile;
            bestResult = result;
        }
    }
    
    return bestResult;
}

ORBDetector::ORBMatchResult ORBDetector::multimatch(const Mat& img, const Mat& templ) const
{
    ORBMatchResult bestResult;
    bestResult.score = -1;
    
    for (double k=1.0 ; k <= 4.0 ; k += 0.5)
    {
        Mat resized;
        resize(templ, resized, Size(0,0), 1/k, 1/k, INTER_NEAREST);
        
        ORBMatchResult result = match(img, resized);
        if (result.score >= 0 && result.score > bestResult.score)
            bestResult = result;
    }
    
    return bestResult;
}

ORBDetector::ORBMatchResult ORBDetector::match(const Mat& img, const Mat& templ) const
{
    //-- Step 1: Detect the keypoints using ORB Detector
    int patch = round(28 * templ.rows / 480.0);
    ORB orb(500, 1.2, 9, patch, 0, 2, ORB::HARRIS_SCORE, patch);

    std::vector<KeyPoint> keypoints_object, keypoints_scene;
    orb.detect( templ, keypoints_object );
    orb.detect( img, keypoints_scene );

    //-- Step 2: Calculate descriptors (feature vectors)
    Mat descriptors_object, descriptors_scene;
    orb.compute( templ, keypoints_object, descriptors_object );
    orb.compute( img, keypoints_scene, descriptors_scene );

    //-- Step 3: Matching descriptor vectors using FLANN matcher
    BFMatcher matcher(NORM_HAMMING);
    std::vector< DMatch > matches;
    matcher.match( descriptors_object, descriptors_scene, matches );

    int n = matches.size();
    if (n < 5)
    {
        ORBMatchResult result;
        result.score = -1;
        return result;
    }
    
    //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist, min 5, max 50)
    std::sort(matches.begin(), matches.end(), matchCompareFn);
    std::vector< DMatch > good_matches;
    for (int i=0 ; i < n && i < 50 ; i++)
    {
        if (i >= 4 && matches[i].distance > 3*matches[0].distance)
            break;
        good_matches.push_back(matches[i]);
    }
    
    //-- Localize the object
    std::vector<Point2f> obj;
    std::vector<Point2f> scene;

    for( int i = 0; i < good_matches.size(); i++ )
    {
        //-- Get the keypoints from the good matches
        obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
        scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
    }
    
    Mat H = findHomography( obj, scene, CV_RANSAC );

    //-- Get the corners from the image_1 ( the object to be "detected" )
    std::vector<Point2f> obj_corners(4);
    obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( templ.cols, 0 );
    obj_corners[2] = cvPoint( templ.cols, templ.rows ); obj_corners[3] = cvPoint( 0, templ.rows );
    std::vector<Point2f> scene_corners(4);

    perspectiveTransform( obj_corners, scene_corners, H);
    Mat warpedImg;
    warpPerspective(img, warpedImg, H, templ.size(), INTER_LINEAR | WARP_INVERSE_MAP);
    
    double logKernel[5][5] = {{0.0448, 0.0468,  0.0564, 0.0468, 0.0448},
                              {0.0468, 0.3167,  0.7146, 0.3167, 0.0468},
                              {0.0564, 0.7146, -4.9078, 0.7146, 0.0564},
                              {0.0468, 0.3167,  0.7146, 0.3167, 0.0468},
                              {0.0448, 0.0468,  0.0564, 0.0468, 0.0448}};
    Mat kernelMat = Mat(5, 5, CV_64F, logKernel);
    
    Mat filteredImg, filteredTemplate;
    filter2D(warpedImg, filteredImg, -1, kernelMat);
    filter2D(templ, filteredTemplate, -1, kernelMat);
    
    Mat binFilteredImg, binFilteredTemplate;
    binFilteredImg = removeIsolatedPixels(binarizeImageKMeans(filteredImg), 2);
    binFilteredTemplate = removeIsolatedPixels(binarizeImageKMeans(filteredTemplate), 2);
    
    Mat binFDImg, binFDTemplate;
    int s = round(16 * templ.rows / 480.0);
    kernelMat = getStructuringElement(MORPH_ELLIPSE, Size(s,s));
    dilate(binFilteredImg, binFDImg, kernelMat);
    dilate(binFilteredTemplate, binFDTemplate, kernelMat);
    
    Mat binDiffImg1 = binFilteredTemplate > binFDImg;
    Mat binDiffImg2 = binFilteredImg > binFDTemplate;
    
    double ref = cv::sum(binFilteredTemplate)[0] / (255.0*binFilteredTemplate.rows*binFilteredTemplate.cols);
    double diff1 = cv::sum(binDiffImg1)[0] / (255.0*binDiffImg1.rows*binDiffImg1.cols);
    double diff2 = cv::sum(binDiffImg2)[0] / (255.0*binDiffImg2.rows*binDiffImg2.cols);
    
    ORBMatchResult result;
    result.score = 1.0 - std::min(1.0, std::max(diff1/ref, diff2/ref));
    for (int i=0 ; i < 4 ; i++)
        result.corners[i] = scene_corners[i];
    Mat planes[3] = {binDiffImg1, binFilteredTemplate, binDiffImg2};
    Mat colorDiffImg;
    merge(planes, 3, result.colorDiffImg);
    
    return result;
}

Mat ORBDetector::drawResult(const cv::Mat& img, const ORBDetector::ORBMatchResult& result) const
{
    Mat colorImg;
    if (img.channels() < 3)
        cvtColor(img, colorImg, CV_GRAY2RGB);
    else
        colorImg = img.clone();
    
    for (int i=0 ; i < 4 ; i++)        
        line(colorImg, result.corners[i] , result.corners[(i+1)%4], Scalar(0, 255, 0), 4);
    
    Mat fullDisplay, colorDiffImg = result.colorDiffImg.clone();
    if (colorDiffImg.rows < colorImg.rows)
        copyMakeBorder(colorDiffImg, colorDiffImg, 0, colorImg.rows-colorDiffImg.rows, 0, 0, BORDER_CONSTANT);
    else if (colorImg.rows < colorDiffImg.rows)
        copyMakeBorder(colorImg, colorImg, 0, colorDiffImg.rows-colorImg.rows, 0, 0, BORDER_CONSTANT);
    
    ROS_INFO("%d", colorDiffImg.rows);
    ROS_INFO("%d", colorImg.rows);
    
    hconcat(colorImg, colorDiffImg, fullDisplay);
    return fullDisplay;
}

Mat ORBDetector::binarizeImageKMeans(const Mat& input)
{
    Mat points = input.clone().reshape(0, input.cols*input.rows);
    points.convertTo(points, CV_32F, 1/255.0);
    Mat labels;
    kmeans(points, 2, labels, TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 10, 1.0), 3, KMEANS_PP_CENTERS);
    if (cv::sum(labels)[0] > input.cols*input.rows/2)
        labels = 1 - labels;
    labels.convertTo(labels, CV_8U, 255);
    return labels.reshape(0, input.rows);
}

Mat ORBDetector::removeIsolatedPixels(const Mat& binInput, int nbMinNeighbours)
{
    Mat binInputCopy = binInput.clone();
    binInputCopy.convertTo(binInputCopy, CV_32F, 1/255.0);
    float kernel[3][3] = {{1, 1, 1},
                          {1, 0, 1},
                          {1, 1, 1}};
    Mat kernelMat = Mat(3, 3, CV_32F, kernel);
    Mat filtered, output;
    filter2D(binInputCopy, filtered, -1, kernelMat);
    threshold(filtered, output, nbMinNeighbours-0.5, 1.0, THRESH_BINARY);
    output = output.mul(binInputCopy);
    output.convertTo(output, CV_8U, 255);
    
    return output;
}

std::vector<Mat> ORBDetector::splitImageAndZoom(const Mat& img, int nbBlocks)
{
    std::vector<cv::Mat> vec;
    double deltaX = img.cols / (double)nbBlocks;
    double deltaY = img.rows / (double)nbBlocks;
    for (double x = 0 ; x <= img.cols-deltaX ; x += deltaX/2)
    {
        int ix = ceil(x);
        for (double y = 0 ; y <= img.rows-deltaY ; y += deltaY/2)
        {
            int iy = ceil(y);
            cv::Mat tile = img(cv::Range(iy, min((int)floor(y+deltaY)+1, img.rows)), cv::Range(ix, min((int)floor(x+deltaX)+1, img.cols)));
            cv::Mat resizedTile ;
            cv::resize(tile, resizedTile, img.size(), 0, 0, CV_INTER_NN);
            vec.push_back(resizedTile);
        }
    }
    return vec;
}

Mat ORBDetector::sharpen(const Mat& img)
{
    Mat output;
    GaussianBlur(img, output, cv::Size(21, 21), 10);
    addWeighted(img, 1.5, output, -0.5, 0, output);
    return output;
}

static bool matchCompareFn(const DMatch& m1, const DMatch& m2)
{
    return m1.distance < m2.distance;
}
    
