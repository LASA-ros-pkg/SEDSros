#include "ros/ros.h"
#include "std_srvs/Empty.h"

#include "seds/SedsOptimize.h"
#include "mldemos/dynamicalSEDS.h"
#include "process_mlfile.hpp"

bool sedsSRV(seds::SedsOptimize::Request  &req, seds::SedsOptimize::Response &res )
{
  DynamicalSEDS seds = DynamicalSEDS();
  vector< vector<fvec> > trajectories;
  float dT;
  ivec labels;

  if (req.filetype == 0){
    // process the input file into trajectories for training
    string filename = req.filenames[0];
    process_dataset_manager_file(filename, trajectories, labels, dT); 
    seds.dT = dT;

    // train the model
    seds.Train(trajectories, labels);

    // save the model
    seds.seds->saveModel(req.outputfile.c_str());
  } else {
    ROS_ERROR("Filetype not supported!");
  }

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
