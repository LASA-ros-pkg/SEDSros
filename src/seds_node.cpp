#include <algorithm>
#include "ros/ros.h"
#include "std_srvs/Empty.h"

#include "seds/SedsOptimize.h"
#include "mldemos/dynamicalSEDS.h"
#include "process_mlfile.hpp"
#include "process_bagfile.hpp"

bool sedsSRV(seds::SedsOptimize::Request  &req, seds::SedsOptimize::Response &res )
{
  DynamicalSEDS seds = DynamicalSEDS();
  vector< vector<fvec> > trajectories;
  float dT;
  ivec labels;
  string filename = req.filename;

  ROS_INFO("Called on filetype: %d", req.filetype);

  if (req.filetype == 0){ // an ml file generated using mldemos

    // process the input file into trajectories for training
    process_dataset_manager_file(filename, trajectories, labels, dT);
    seds.dT = dT;

    // train the model
    seds.Train(trajectories, labels);

    // save the model
    seds.seds->saveModel(req.outputfile.c_str());
  } else if (req.filetype == 1){ // a bag file

    // process the input file into trajectories for training
    process_bagfile(filename, trajectories, labels, dT);
    seds.dT = dT;

    // train the model
    seds.Train(trajectories, labels);

    // save the model
    seds.seds->saveModel(req.outputfile.c_str());
  } else if (req.filetype == 2){ // a model is already provided
    ROS_ERROR("Not implemented yet!");
  } else {
    ROS_ERROR("Filetype not supported!");
  }

  // some initial model testing...
  fvec sample;
  sample.push_back(3.0);
  sample.push_back(3.0);
  
  fvec result = seds.Test(sample);
  for (int i=0;i<result.size();i++){
    ROS_INFO("res[%d]: %f", i, result[i]);
  }

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
