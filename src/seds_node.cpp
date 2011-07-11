#include <algorithm>
#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "seds/SedsOptimize.h"
#include "seds/SedsModel.h"
#include "seds/FileIO.h"
#include "seds_wrapper.hpp"

SEDS *seds_obj = NULL;
string source_fid;
string target_fid;

bool saveFileSRV(seds::FileIO::Request &req, seds::FileIO::Response &res)
{

  if (!seds_obj){
    ROS_INFO("You need to run seds_optimize to generate new model parameters!");
    return false;
  }

  rosbag::Bag bag;
  seds::ModelParameters model;
  populate_model_msg(seds_obj, source_fid, target_fid, model);


  bag.open(req.filename.c_str(), rosbag::bagmode::Write);
  bag.write("seds/params", ros::Time::now(), model);

  bag.close();
  return true;
}

bool getParamsSRV(seds::SedsModel::Request &req, seds::SedsModel::Response &res)
{
  // helper function in seds_wrapper
  populate_model_msg(seds_obj, source_fid, target_fid, res.model);

  return true;
}

bool sedsSRV(seds::SedsOptimize::Request  &req, seds::SedsOptimize::Response &res )
{
  vector< vector<fvec> > trajectories;
  float dT;
  ivec labels;
  string filename = req.filename;

  // need to know what transforms were used to generate the pose data
  if (seds_obj){
    delete seds_obj;
  }
  seds_obj = new SEDS();

  // process the input file into trajectories for training
  process_bagfile(filename, trajectories, dT, source_fid, target_fid);

  // set up and perform seds optimization
  seds_optimize(seds_obj, trajectories, dT);

  ROS_INFO("Optimization done!");
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "seds_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("/seds/optimize", sedsSRV);
  ros::ServiceServer modelparams = n.advertiseService("/seds/params", getParamsSRV);
  ros::ServiceServer savefile = n.advertiseService("/seds/save_file", saveFileSRV);

  ROS_INFO("Ready to optimize.");
  ros::spin();

  return 0;
}
