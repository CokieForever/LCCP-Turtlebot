
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace cv;
using namespace std;

Mat src; Mat star;
int lower_sat = 100;
int max_hue = 179;
RNG rng(12345);

/// Function header
void thresh_callback(int, void* );

/** @function main */
int main( int argc, char** argv )
{
  /// Load source images
  src = imread( argv[1], 1 );
  star = imread( argv[2], 1 );



  /// Create Window
  char* source_window = "Source";
  namedWindow( source_window, CV_WINDOW_AUTOSIZE );
  imshow( source_window, src );

  createTrackbar( " Hue sat:", "Source", &lower_sat, max_hue, thresh_callback );
  thresh_callback( 0, 0 );

  waitKey(0);
  return(0);
}

/** @function thresh_callback */
void thresh_callback(int, void* )
{
  Mat src_image, src_output;
  Mat tmpl_img, tmpl_output;
  vector<vector<Point> > srcCont;
  vector<vector<Point> > tmplCont;
  vector<Vec4i> hierarchy;
  vector<Vec4i> hierarchy2;

  star.copyTo(tmpl_img);
  src.copyTo(src_image);
  cv::cvtColor(src, src_image, cv::COLOR_BGR2HSV);
  cv::cvtColor(tmpl_img, tmpl_img, cv::COLOR_BGR2HSV);
  inRange(src_image, cv::Scalar(20,220,220), cv::Scalar(30, 255, 255), src_output);
  inRange(tmpl_img, cv::Scalar(20,220,220), cv::Scalar(30, 255, 255), tmpl_output);
  findContours(src_output, srcCont, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_TC89_KCOS, Point(0.0) );
  findContours(tmpl_output, tmplCont, hierarchy2, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_TC89_KCOS, Point(0.0) );


  int contSizeSrc= srcCont.size();
  //std::cout<<contSizeSrc<<endl;
  int contSizeTmpl = tmplCont.size();
  std::cout<<contSizeTmpl<<endl;
  double match;
  std::cout<<tmplCont.data()[0]<<endl;
  vector<int> matches;
  for(int i=0; i<contSizeSrc; i++)
   {
     for(int j=0; j<contSizeTmpl;j++)
       {
         match = matchShapes(srcCont.data()[i], tmplCont.data()[j], 2, 0);
         if(match<0.01 )
           {
             matches.push_back(i);
             std::cout<<srcCont.data()[i]<<endl;
           }
       }
   }
  Mat drawing = Mat::zeros( src_output.size(), CV_8UC3 );
  std::cout<<matches.size()<<endl;
  //std::cout<<secondContours.size()<<endl;
  vector<vector<Point> > contourOutput;
  for(int j=0; j<matches.size(); j++)
    {
      contourOutput.push_back(srcCont.data()[j]);
    }
     std::cout<<"contourOutput generated"<<endl;
  for( int i = 0; i< matches.size(); i++ )
     {
       Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
       drawContours( drawing, contourOutput, i, color, 2, 8, hierarchy, 0, Point() );
     }


  /// Show in a window
  namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
  imshow( "Contours", drawing );



  char* result_window = "Result";
  namedWindow( result_window, CV_WINDOW_AUTOSIZE);
  imshow(result_window, src_output);
/*
  char* buff_window = "window";
  namedWindow( buff_window, CV_WINDOW_AUTOSIZE);
  imshow(buff_window, tmpl_output);*/
}
