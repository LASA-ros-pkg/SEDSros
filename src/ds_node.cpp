#include "std_srvs/Empty.h"
#include "fgmm/fgmm++.hpp"
#include "seds_wrapper.hpp"
#include "SEDS.h"
#include "ros/ros.h"
#include "seds/DSSrv.h"
#include "seds/SedsModel.h"
#include "seds/FileIO.h"
#include "seds/DSLoaded.h"
#include "std_srvs/Empty.h"
#include <sys/stat.h>
Gmm *gmm = NULL;
fvec endpoint;
float dT;

bool loadFileSRV(seds::FileIO::Request &req, seds::FileIO::Response &res)
{
  SEDS *seds = new SEDS();

  ROS_INFO("Loading model: %s", req.filename.c_str());

  // check for file existance
  struct stat stFileInfo;
  int intStat = stat(req.filename.c_str(),&stFileInfo);
  if (intStat != 0){
    ROS_INFO("File does not exist!");
    return false;
  }

  seds->loadModel(req.filename.c_str());
  int dim = seds->d * 2;
  int nbClusters = seds->K;
  dT = seds->dT;

  endpoint.resize(dim);
  for(int i = 0; i < dim; i++){
    endpoint[i] = seds->Offset(i);
  }

  ROS_INFO("Using endpoint: %f %f %f", endpoint[0], endpoint[1], endpoint[2]);
  ROS_INFO("Using dT: %f", dT);

  if (gmm != NULL)
    delete gmm;

  gmm = new Gmm(nbClusters,dim);

  // and we copy the values back to the source
  float *mu = new float[dim];
  float *sigma = new float[dim*dim];

  for(int i = 0; i < nbClusters; i++){
    for (int d1 = 0; d1 < dim; d1++){
      mu[d1] = seds->Mu(d1, i);
      for (int d2 = 0; d2 < dim; d2++){
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

bool getParamsSRV(seds::SedsModel::Request &req, seds::SedsModel::Response &res)
{
  int d;

  if (gmm == NULL){
    ROS_INFO("You need to load parameters first!");
    return false;
  }

  res.model.dim = gmm->dim;
  res.model.ncomp = gmm->nstates;
  res.model.dT = dT;
  d = res.model.dim;

  res.model.offset.resize(d);
  for (int i = 0; i < d; i++){
    res.model.offset[i] = endpoint[i];
  }

  res.model.priors.resize(res.model.ncomp);
  for (int k = 0; k < res.model.ncomp; k++){
    res.model.priors[k] = fgmm_get_prior(gmm->c_gmm, k);
  }

  res.model.mus.resize(d * res.model.ncomp);
  for(int k = 0; k < res.model.ncomp; k++){
    for (int i = 0; i < d; i++){
      res.model.mus[ k * d + i ] = fgmm_get_mean(gmm->c_gmm, k)[i];
    }
  }

  res.model.sigmas.resize(d * d * res.model.ncomp);
  for(int k = 0; k < res.model.ncomp; k++){
    for (int i = 0; i < d; i++){
      for (int j = 0; j < d; j++){
	res.model.sigmas[k * d * d + i * d + j] = fgmm_get_covar_smat(gmm->c_gmm,k)[i *d + j];
      }
    }
  }

  return true;
}


bool loadSRV(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{

  ROS_INFO("Loading parameters from running SEDS NODE.");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<seds::SedsModel>("/seds/params");
  seds::SedsModel srv;

  // make the call to seds for the parameters
  client.call(srv);

  // now extract the parameters for the gmm!
  int dim = srv.response.model.dim * 2;
  int nbClusters = srv.response.model.ncomp;
  dT = srv.response.model.dT;

  endpoint.resize(dim);
  for(int i = 0; i < dim; i++){
    endpoint[i] = srv.response.model.offset[i];
  }

  ROS_INFO("Using endpoint: %f %f %f", endpoint[0], endpoint[1], endpoint[2]);
  ROS_INFO("Using dT: %f", dT);

  if (gmm != NULL)
    delete gmm;

  gmm = new Gmm(nbClusters,dim);

  // and we copy the values back to the source
  float *mu = new float[dim];
  float *sigma = new float[dim*dim];

  for(int k = 0; k < nbClusters; k++){
    for (int i = 0; i < dim; i++){
      mu[i] = srv.response.model.mus[k * dim + i];
      for (int j = 0; j < dim; j++){
	sigma[i * dim + j] = srv.response.model.sigmas[k * dim * dim + i * dim + j];
      }
    }
    fgmm_set_prior(gmm->c_gmm, k, srv.response.model.priors[k]);
    fgmm_set_mean(gmm->c_gmm, k, mu);
    fgmm_set_covar(gmm->c_gmm, k, sigma);
  }

  delete [] sigma;
  delete [] mu;

  gmm->initRegression(dim/2);

  return true;
}

bool isLoadedSRV(seds::DSLoaded::Request &req, seds::DSLoaded::Response &res){

  if (gmm == NULL){
    res.loaded = false;
  } else {
    res.loaded = true;
  }

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

  for(int i = 0; i < dim; i++){
    x[i] = (req.x[i] - endpoint[i])* 1000.f; // offset and scale values
  }

  ROS_DEBUG("Performing regression using SEDS parameters!");

  gmm->doRegression(x, velocity, sigma);

  for(int i = 0; i < dim; i++){
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

  ros::ServiceServer load_model_service = n.advertiseService("/ds_node/load_model", loadSRV);
  ros::ServiceServer load_file_service = n.advertiseService("/ds_node/load_file", loadFileSRV);
  ros::ServiceServer ds_service = n.advertiseService("/ds_node/ds_server", dsSRV);
  ros::ServiceServer ds_loaded = n.advertiseService("/ds_node/is_loaded", isLoadedSRV);
  ros::ServiceServer ds_params = n.advertiseService("/ds_node/params", getParamsSRV);

  ROS_INFO("Ready to control.");
  ros::spin();

  return 0;
}
