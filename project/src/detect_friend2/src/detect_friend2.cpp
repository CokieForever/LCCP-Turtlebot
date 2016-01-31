#include "detect_friend2.h"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <ros/ros.h>


using namespace cv;
using namespace std;
int main()
{


     Mat img= imread("/home/ros/Desktop/ref/coin2.png");
    DetectFriend2 df;
    df.clusters(img);
}


DetectFriend2::DetectFriend2()
{

}

//DetectFriend2::Rect
std::vector<Rect> DetectFriend2::clusters(Mat img)
{

  cv::Mat image_scene, image_scene2, image_scene3, image_scene4 = Mat(image_scene.rows, image_scene.cols, CV_64FC3);
  cv::Mat diff;

  //Mat img=imread("/home/ros/Desktop/ref/test6.png");
  int i, j;
  //Scalar yellowmin(0, 130, 200);
 // Scalar yellowmax(30,255,255);
 // Scalar redmin(0,0,100);
//  Scalar redmax(30,30,250);
//  Scalar yellowmin(194,147,153);
//  Scalar yellowmax(220, 107,248);
  Scalar yellowcoinmin(0, 210, 250);
  Scalar yellowcoinmax(90, 255, 255);
 // cvtColor(img, img, CV_BGR2Lab);
  inRange(img, yellowcoinmin, yellowcoinmax, diff);



imshow("diff", diff);
ROS_INFO("1");
int type=img.type();
ROS_INFO("type %d", type);
int type2=diff.type();
ROS_INFO("type2 %d", type2);




  SimpleBlobDetector::Params params;

  // Change thresholds
  //params.minThreshold = 255;
  //params.maxThreshold = 0;
  params.filterByColor=true;
  params.blobColor=255;
  // Filter by Area.
  params.filterByArea = true;
  params.minArea = 200;

  // Filter by Circularity
  params.filterByCircularity = false;
  //params.minCircularity = 0.1;

  // Filter by Convexity
  params.filterByConvexity = false;
  //params.minConvexity = 0.87;

  // Filter by Inertia
  params.filterByInertia = false;
  //params.minInertiaRatio = 0.01;



    // Set up detector with params
 SimpleBlobDetector detector(params);
vector<vector<Point> > contours;
vector<Vec4i> hierarchy;
RNG rng(12345);
  //Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
  std::vector<KeyPoint> keypoints;
  detector.detect( diff, keypoints);
  ROS_INFO("keypoint size: %lu", keypoints.size());
  Mat im_with_keypoints;
  drawKeypoints( diff, keypoints, im_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DEFAULT );
  imshow("keypoints", im_with_keypoints );
  findContours( diff, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
 // std::sort(contours.begin(), contours.end(), compareContourAreas);

 vector<vector<Point> > contours_poly( contours.size() );
 vector<Rect> boundRect( contours.size() );
 Mat drawing = Mat::zeros( diff.size(), CV_8UC3 );

for(int i=0; i<contours.size(); i++)
  {

    if (contourArea(contours[i],true) > 0)
      {cout << "black" << endl;
       contours.erase(contours.begin() + i);
       hierarchy.erase(hierarchy.begin()+i);
       i--;
      }
    else
      {cout << "white" << endl;}
  }



ROS_INFO("contour size %lu", contours.size() );
double area;
for( int i = 0; i < contours.size(); i++ )
    { approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
      area=contourArea(contours_poly[i]);
       {cout << "area" <<area<< endl;}
      if (area<100)
        {
         contours.erase(contours.begin() + i);
         hierarchy.erase(hierarchy.begin()+i);
         i--;
        }
      else
        {
         boundRect[i] = boundingRect( Mat(contours_poly[i]) );
         Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
         rectangle( img,   boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
        }
    }

for( int i = 0; i< contours.size(); i++ )
   {
     Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
     drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
     rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
     ROS_INFO("tl %d", boundRect[i].br().x );

   }


 imshow( "Contours", drawing );
 imshow("final", img);
 waitKey(0);
 return boundRect;
}


