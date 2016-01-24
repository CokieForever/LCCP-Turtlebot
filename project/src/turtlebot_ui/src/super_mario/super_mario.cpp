#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>

using namespace std;
using namespace cv;

/// Global Variables
Mat img; Mat templ; Mat result; Mat newResult;
char* image1_window = "fucking Image";
char* image_window = "Source Image";
char* result_window = "Result window";

int match_method;
int max_Trackbar = 5;

/// Function Headers
void MatchingMethod( int, void* );

/** @function main */
int main( int argc, char** argv )
{
  /// Load image and template
  img = imread( argv[1], 1 );
  templ = imread( argv[2], 1 );

  /// Create windows
  namedWindow( image1_window, CV_WINDOW_AUTOSIZE );
  namedWindow( image_window, CV_WINDOW_AUTOSIZE );
  namedWindow( result_window, CV_WINDOW_AUTOSIZE );

  /// Create Trackbar
 // char* trackbar_label = "Method: \n 0: SQDIFF \n 1: SQDIFF NORMED \n 2: TM CCORR \n 3: TM CCORR NORMED \n 4: TM COEFF \n 5: TM COEFF NORMED";
 // createTrackbar( trackbar_label, image_window, &match_method, max_Trackbar, MatchingMethod );
  match_method = 4;
  MatchingMethod( 0, 0 );

  waitKey(0);
  return 0;
}

/**
 * @function MatchingMethod
 * @brief Trackbar callback
 */
void MatchingMethod( int, void* )
{
  /// Source image to display
  Mat img_display;
 img.copyTo(img_display);
  //Mat img_temp;
 // cvtColor( img, img_display, CV_BGR2GRAY );
//cvtColor( templ, img_display, CV_BGR2GRAY );
  /// Create the result matrix
  int result_cols =  img_display.cols - templ.cols + 1;
  int result_rows = img_display.rows - templ.rows + 1;

  result.create( result_rows, result_cols, CV_8UC1);

  /// Do the Matching and Normalize
  matchTemplate( img, templ, result, match_method );
  normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );

  /// Localizing the best match with minMaxLoc
  double minVal; double maxVal; Point minLoc; Point maxLoc;
  Point matchLoc;

  minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );


  /// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
  if( match_method  == 2 || match_method  == 3 )
    { matchLoc = minLoc; }
  else
    { matchLoc = maxLoc; }

  cv::Mat subArray(img, Rect(matchLoc.x, matchLoc.y, templ.rows*2, templ.cols*2));
  std::cout<<subArray.rows<<" und "<<subArray.cols<<endl;
imshow( image1_window, subArray );

  int newResult_rows = (subArray.rows-templ.rows+1);
  int newResult_cols = (subArray.cols - templ.cols+1);
  newResult.create( newResult_rows,newResult_cols , CV_8UC1);
 // for(int i =0; i<result.rows;i++){for(int j =0; j<result.cols;j++) std::cout<<(int)result.row(i).data[j]<<endl;}
  std::cout<<matchLoc.x<<" und "<<matchLoc.y<<endl;

  //cv::Mat subArray = img(cv::Range(matchLoc.x, matchLoc.y), cv::Range(matchLoc.x+templ.rows, matchLoc.y+templ.cols));
  matchTemplate( subArray, templ, result, match_method );
  std::cout<<"test"<<endl;
 for(int i =0; i<newResult.rows;i++){for(int j =0; j<newResult.cols;j++) std::cout<<(int)newResult.row(i).data[j]<<endl;}

  /// Show me what you got
  rectangle( img_display, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(0), 2, 8, 0 );
  rectangle( result, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(0), 2, 8, 0 );

  //imshow( image_window, img_display );
  imshow( result_window, result );

  return;
}
