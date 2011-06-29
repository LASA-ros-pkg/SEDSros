#include <algorithm>
#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "seds/SedsOptimize.h"
#include "seds_wrapper.hpp"

bool sedsSRV(seds::SedsOptimize::Request  &req, seds::SedsOptimize::Response &res )
{
  vector< vector<fvec> > trajectories;
  float dT;
  ivec labels;
  string filename = req.filename;
  SEDS *seds = new SEDS();

  // process the input file into trajectories for training
  process_bagfile(filename, trajectories, dT);

  // set up and perform seds optimization
  seds_optimize(seds, trajectories, dT);

  // save the model
  seds->saveModel(req.outputfile.c_str());

  delete seds;
  ROS_INFO("All done!");
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
