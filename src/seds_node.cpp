#include <algorithm>
#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "seds/SedsOptimize.h"
#include "seds/SedsModel.h"
#include "seds/FileIO.h"
#include "seds_wrapper.hpp"

SEDS *seds_obj = NULL;

bool saveFileSRV(seds::FileIO::Request &req, seds::FileIO::Response &res)
{

  if (!seds_obj){
    ROS_INFO("You need to run seds_optimize to generate new model parameters!");
    return false;
  }

  seds_obj->saveModel(req.filename.c_str());

  return true;
}

bool getParamsSRV(seds::SedsModel::Request &req, seds::SedsModel::Response &res)
{
  int d;

  if (!seds_obj){
    ROS_INFO("You need to run seds_optimize to generate new model parameters!");
    return false;
  }

  res.model.dim = seds_obj->d;
  res.model.ncomp = seds_obj->K;
  res.model.dT = seds_obj->dT;
  d = res.model.dim * 2;

  res.model.offset.resize(d);
  for (int i = 0; i < d; i++){
    res.model.offset[i] = seds_obj->Offset(i);
  }

  res.model.priors.resize(res.model.ncomp);
  for (int k = 0; k < res.model.ncomp; k++){
    res.model.priors[k] = seds_obj->Priors(k);
  }

  res.model.mus.resize(d * res.model.ncomp);
  for(int k = 0; k < res.model.ncomp; k++){
    for (int i = 0; i < d; i++){
      res.model.mus[ k * d + i ] = seds_obj->Mu(i,k);
    }
  }

  res.model.sigmas.resize(d * d * res.model.ncomp);
  for(int k = 0; k < res.model.ncomp; k++){
    for (int i = 0; i < d; i++){
      for (int j = 0; j < d; j++){
	res.model.sigmas[k * d * d + i * d + j] = seds_obj->Sigma[k](i,j);
      }
    }
  }

  return true;
}

bool sedsSRV(seds::SedsOptimize::Request  &req, seds::SedsOptimize::Response &res )
{
  vector< vector<fvec> > trajectories;
  float dT;
  ivec labels;
  string filename = req.filename;

  if (seds_obj){
    delete seds_obj;
  }
  seds_obj = new SEDS();

  // process the input file into trajectories for training
  process_bagfile(filename, trajectories, dT);

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
  ros::ServiceServer savefile = n.advertiseService("/seds/savefile", saveFileSRV);

  ROS_INFO("Ready to optimize.");
  ros::spin();

  return 0;
}
