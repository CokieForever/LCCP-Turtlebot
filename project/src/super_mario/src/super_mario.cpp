#include "super_mario.hpp"

using namespace std;
using namespace cv;



SuperMario::SuperMario(ros::NodeHandle &nh, Mat &tmpl) : m_nh(nh), m_templ(tmpl)
{
  m_checkcoin = false;
  m_imageSub = m_nh.subscribe("/camera/rgb/image_rect_color",1, &SuperMario::imageCallback, this);
  //subscribe to topic from movement
  m_movementSub = m_nh.subscribe("/super_mario/check_object", 1, &SuperMario::movementCallback, this);

  ROS_INFO("Subscribed");

}
void SuperMario::movementCallback(std_msgs::Bool move_bool)
{
    // ********************   2     *******************
    m_checkcoin =  (move_bool.data) ? false : true;
}

void SuperMario::imageCallback(const sensor_msgs::ImageConstPtr& img_msg)
{
   ROS_INFO("Image Callback");
  Mat mat_img;
  try
  {
      cv_bridge::CvImageConstPtr imgPtr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
      if (imgPtr == NULL)
      {
          ROS_WARN("Received NULL image.");
          return;
      }
      mat_img = imgPtr->image;
  }
  catch (cv_bridge::Exception& ex)
  {
      ROS_ERROR("cv_bridge exception: %s", ex.what());
      exit(-1);
  }
   ROS_INFO("img ptr created");
  /// Create windows
  ///
  const char* image1_window = "fucking Image";
  const char* image_window = "Source Image";
  const char* result_window = "Result window";

  namedWindow( image1_window, CV_WINDOW_AUTOSIZE );
  namedWindow( image_window, CV_WINDOW_AUTOSIZE );
  namedWindow( result_window, CV_WINDOW_AUTOSIZE );
 ROS_INFO("Matching method");
 // m_match_method = 4;
 // SuperMario::MatchingMethod( 0, 0 );

 // ********************   3     *******************
 if (m_checkcoin)
 {
    //check if there's a coin in the received image
     m_img = mat_img; //assign to member variable, so matching method can use it.
     SuperMario::MatchingMethod(0,0);

 }
 ROS_INFO("After matching");
  waitKey(0);

  imshow("Detection", mat_img);

  //save img - TEMPORARY TO TAKE SCREENSHOT OF TEMPLATE
  vector<int> compression_params; //vector that stores the compression parameters of the image

  compression_params.push_back(CV_IMWRITE_JPEG_QUALITY); //specify the compression technique

  compression_params.push_back(98); //specify the compression quality
   ROS_INFO("Taking a screenshot.");
  bool bSuccess = imwrite("/home/alex/Desktop/screenshots/TestImage.jpg", mat_img, compression_params); //write the image to file

  if (!bSuccess)
  {
    cout << "Could not save image!" << endl;
  }
}

// ********************   4     *******************
void SuperMario::MatchingMethod( int, void* )
{
  //// Source image to display
  Mat img_display;
  m_img.copyTo( img_display );

  /// Create the result matrix
  int result_cols =  m_img.cols - m_templ.cols + 1;
  int result_rows = m_img.rows - m_templ.rows + 1;

  m_result.create( result_rows, result_cols, CV_32FC1 );

  /// Do the Matching and Normalize
  matchTemplate( m_img, m_templ, m_result, m_match_method );
  normalize( m_result, m_result, 0, 1, NORM_MINMAX, -1, Mat() );

  /// Localizing the best match with minMaxLoc
  double minVal; double maxVal; Point minLoc; Point maxLoc;
  Point matchLoc;

  minMaxLoc( m_result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );

  /// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
  if( m_match_method  == CV_TM_SQDIFF || m_match_method == CV_TM_SQDIFF_NORMED )
    { matchLoc = minLoc; }
  else
    { matchLoc = maxLoc; }

  /// Show me what you got
  rectangle( img_display, matchLoc, Point( matchLoc.x + m_templ.cols , matchLoc.y + m_templ.rows ), Scalar::all(0), 2, 8, 0 );
  rectangle( m_result, matchLoc, Point( matchLoc.x + m_templ.cols , matchLoc.y + m_templ.rows ), Scalar::all(0), 2, 8, 0 );

  imshow( "Source Image", img_display );
  imshow( "Results", m_result );

  return;
}

void SuperMario::Start()
{
  ROS_INFO("Starting");
  ros::spin();
}

