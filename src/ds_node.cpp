#include "std_srvs/Empty.h"
#include "fgmm/fgmm++.hpp"
#include "mldemos/SEDS.h"
#include "mldemos/public.h"
#include "ros/ros.h"
#include "seds/DSLoad.h"
#include "seds/DSSrv.h"

Gmm *gmm = NULL;

bool loadSRV(seds::DSLoad::Request &req, seds::DSLoad::Response &res){

  // load model parameters
  SEDS *seds = new SEDS();

  ROS_INFO("Loading model: %s", req.filename.c_str());

  seds->loadModel(req.filename.c_str());
  int dim = seds->d * 2;
  int nbClusters = seds->K;

  gmm = new Gmm(nbClusters,dim);

  // and we copy the values back to the source
  float *mu = new float[dim];
  float *sigma = new float[dim*dim];

  FOR(i, nbClusters)
    {
      FOR(d, dim) mu[d] = seds->Mu(d, i);
      FOR(d1, dim)
	{
	  FOR(d2, dim)
	    {
	      sigma[d2*dim + d1] = seds->Sigma[i](d1, d2);
	    }
	}
      fgmm_set_prior(gmm->c_gmm, i, seds->Priors(i));
      fgmm_set_mean(gmm->c_gmm, i, mu);
      fgmm_set_covar(gmm->c_gmm, i, sigma);
    }

  delete [] sigma;
  delete [] mu;
  delete seds;

  gmm->initRegression(dim/2);

  return true;
}

bool dsSRV(seds::DSSrv::Request &req, seds::DSSrv::Response &res){

  // compute next move

  if (gmm == NULL){
    ROS_INFO("Gmm not initialized! Call load model first!");
    return true;
  }

  int dim = req.x.size();
  res.dx.resize(dim);

  float *x = new float[dim];
  float *velocity = new float[dim];
  float *sigma = new float[dim*(dim+1)/2];


  FOR (i, dim){
    x[i] = req.x[i] * 1000.f; // scale values
  }

  ROS_INFO("Performing regression using SEDS parameters!");

  gmm->doRegression(x, velocity, sigma);

  FOR (i, dim){
    res.dx[i] = velocity[i] / 1000.f;
  }

  delete [] x;
  delete [] velocity;
  delete [] sigma;

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ds_server");
  ros::NodeHandle n;

  ros::ServiceServer load_model_service = n.advertiseService("load_model", loadSRV);
  ros::ServiceServer ds_service = n.advertiseService("ds_server", dsSRV);

  ROS_INFO("Ready to control.");
  ros::spin();

  return 0;
}
