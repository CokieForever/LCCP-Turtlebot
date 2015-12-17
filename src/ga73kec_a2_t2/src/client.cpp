#include "ros/ros.h"
#include "ga73kec_a2_t2/SpawnTurtle.h"
#include <cstdlib>

int main(int argc, char**argv)
{
  ros::init(argc,argv,"client");
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<ga73kec_a2_t2::AddTwoInts>("add_two_ints");
  ga73kec_a2_t2::AddTwoInts srv;
  srv.request.a = 4;
  srv.request.b = 5;
  if (client.call(srv))
  {
    ROS_INFO("The sum is %ld", (long int) srv.response.sum);
  }
  return 0;
}
