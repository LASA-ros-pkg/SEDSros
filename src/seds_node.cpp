#include "ros/ros.h"
#include "seds/SedsOptimize.h"

bool sedsSRV(seds::SedsOptimize::Request  &req, seds::SedsOptimize::Response &res )
{
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "seds_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("seds_server", sedsSRV);
  ROS_INFO("Ready to optimize.");
  ros::spin();

  return 0;
}
