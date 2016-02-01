
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <ros/ros.h>


using namespace cv;
using namespace std;
int main()
{

  Mat templ= imread("/home/ros/Desktop/ref/ref.png");
  Mat img=imread("/home/ros/Desktop/ref/test6.png");


  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;

  Mat drawing = Mat::zeros( img.size(), CV_8UC3 );
  RNG rng(12345);
  double area;

////DetectFriend2::Rect
//void DetectFriend2::clusters(const Mat& img, const Mat& templ)
//{
  cv::Mat image_object, image_object2;
  cv::Mat image_scene;
  cv::Mat image_scene2;
  vector<Mat> rgb1, rgb2;
  Mat add_res, add_res2, euclidean1, euclidean2;
  Mat imgCopysc=img.clone();
  Mat imgCopytemp=templ.clone();
  imgCopysc.convertTo(imgCopysc, CV_32FC3);
  imgCopytemp.convertTo(imgCopytemp, CV_32FC3);
  int i, j;
  Scalar yellow(0, 237, 237);




//int type2=yellow.type();


 absdiff(imgCopysc, yellow, image_scene);
// int  type1 = image_scene.type();
//  // subtract(img, yellow, image_scene);
// //  image_scene=abs(image_scene);
//   ROS_INFO("mistake2");
//   ROS_INFO("type1 %d", type);
//   ROS_INFO("type2 %d", type1);
////   ROS_INFO("type3", type2);


   imshow("Image_scene", image_scene);
   ROS_INFO("2");
   absdiff(imgCopytemp, yellow , image_object);
   imshow("Image_object", image_object);
   ROS_INFO("3");
   ROS_INFO("IMAGE CHANNELS %d", image_object.channels());
   pow(image_object, 2, image_object2);

   pow(image_scene, 2, image_scene2);
  // ROS_INFO("absdifpow %.3f", image_scene2.at<double>(0,0));
   split(image_object2, rgb1);
   split(image_scene2, rgb2);
   add_res=rgb1[0]+rgb1[1]+rgb1[2];
   imshow("rgb1", rgb1[0]);
   imshow("rgb2", rgb1[1]);
   imshow("rgb3", rgb1[2]);
   add_res2=rgb2[0]+rgb2[1]+rgb2[2];
   //add_res.convertTo(add_res, CV_64F);
   //add_res2.convertTo(add_res2, CV_64F);
   sqrt(add_res, euclidean1);
   sqrt(add_res2, euclidean2);
   ROS_INFO("4");
   imshow("euclidean1", euclidean1);

   imshow("euclidean2", euclidean2);
   ROS_INFO("12");




   ROS_INFO("13");
  euclidean1=(euclidean1/(441));
   imshow("result1", euclidean1);
   ROS_INFO("14");
   euclidean2=(euclidean2/441);

   imshow("result2", euclidean2);

   ROS_INFO("15");

   //threshold( euclidean1, dst1, 38 , 255, THRESH_BINARY );
   ROS_INFO("16");
  // threshold( euclidean2, dst2, 38 , 255, THRESH_BINARY );

 // imshow("Image1", dst1);
 // imshow("Image2", dst1);
  Mat scene, obj;


  threshold(euclidean1, obj, 0.20, 1, THRESH_BINARY_INV);
  threshold(euclidean2, scene, 0.20, 1, THRESH_BINARY_INV);
  imshow("obj", obj);
  imshow("scene", scene);

  //find contours in the image.
  Mat euclideanCopy2=scene.clone();
  euclideanCopy2.convertTo(euclideanCopy2, CV_8U);

  int typeeuclidean=euclideanCopy2.type();
  ROS_INFO("euclidean type %d", typeeuclidean);
  findContours( euclideanCopy2, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

  //ignore the black contours
  for(int i=0; i<contours.size(); i++)
   {
    if (contourArea(contours[i],true) > 0)
      {
       ROS_INFO("black");
       contours.erase(contours.begin() + i);
       hierarchy.erase(hierarchy.begin()+i);
       i--;
      }
    else
      {
        ROS_INFO("white");
      }
  }

  ROS_INFO("contour size %lu", contours.size() );
  vector<vector<Point> > contours_poly( contours.size() );
  vector<Rect> boundRect( contours.size() );

  //create rectangles around contours of interest and draw
  for( int i = 0; i < contours.size(); i++ )
    {
      approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
      area=contourArea(contours_poly[i]);
      ROS_INFO("area %.3f", area );
      //ignore the contours that have area less than 10*10
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
         drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
         rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
        }
    }

 imshow( "Contours", drawing );
 imshow("final", img);
 waitKey(0);
 //return vector of rectangles



}


