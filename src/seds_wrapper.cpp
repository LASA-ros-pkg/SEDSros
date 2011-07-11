#include "seds_wrapper.hpp"

void populate_model_msg(SEDS *seds_obj, string source_fid, string target_fid, seds::ModelParameters &model){
  int d;

  if (!seds_obj){
    ROS_INFO("seds_obj is not set!");
    return;
  }

  model.dim = seds_obj->d;
  model.ncomp = seds_obj->K;
  model.dT = seds_obj->dT;
  d = model.dim * 2;

  model.offset.resize(d);
  for (int i = 0; i < d; i++){
    model.offset[i] = seds_obj->Offset(i);
  }

  model.priors.resize(model.ncomp);
  for (int k = 0; k < model.ncomp; k++){
    model.priors[k] = seds_obj->Priors(k);
  }

  model.mus.resize(d * model.ncomp);
  for(int k = 0; k < model.ncomp; k++){
    for (int i = 0; i < d; i++){
      model.mus[ k * d + i ] = seds_obj->Mu(i,k);
    }
  }

  model.sigmas.resize(d * d * model.ncomp);
  for(int k = 0; k < model.ncomp; k++){
    for (int i = 0; i < d; i++){
      for (int j = 0; j < d; j++){
	model.sigmas[k * d * d + i * d + j] = seds_obj->Sigma[k](i,j);
      }
    }
  }

  model.source_fid = source_fid;
  model.target_fid = target_fid;
}

void seds_optimize(SEDS *seds, vector< vector< fvec > > trajectories, float dT){
  vector<fvec> samples;
  fvec endpoint = fvec();
  Gmm *gmm;
  int nclusters = 2;
  int dim = trajectories[0][0].size();
  float *data; // for gmm
  double *ddata; // for seds

  // compute the average endpoint of all the sample trajectories
  endpoint.resize(dim,0.f);
  for (int i = 0; i < trajectories.size(); i++){
    for (int j = 0; j < dim; j++){
      endpoint[j] = endpoint[j] + trajectories[i][trajectories[i].size()-1][j];
    }
  }
  for (int j = 0; j < dim; j++){
    endpoint[j] = endpoint[j] / trajectories.size();
  }

  for (int i = 0; i < trajectories.size(); i++){
    for (int j = 0; j < trajectories[i].size(); j++){
      fvec tmp = fvec();
      tmp.resize(dim, 0.f);
      for (int k = 0; k < dim; k++){
	tmp[k] = trajectories[i][j][k] - endpoint[k];
      }
      samples.push_back(tmp);
    }
  }


  gmm = new Gmm(nclusters, dim);
  data = new float[samples.size() * dim];
  ddata = new REALTYPE[samples.size() * dim];
  for (int i = 0; i < samples.size(); i++){
    for (int j = 0; j < dim; j++){
      data[i * dim + j] = samples[i][j]*1000.f;
      ddata[j * samples.size() + i] = samples[i][j]*1000.f;
    }
  }

  gmm->init(data, samples.size(), 0); // random
  //gmm->init(data, samples.size(), 2); // kmeans
  //gmm->em(data, samples.size(), 1e-4, COVARIANCE_FULL); // em init

  seds->Data = Matrix(ddata, dim, samples.size());
  seds->Offset.Resize(dim/2);
  for (int i = 0; i < dim; i++){
    seds->Offset(i) = endpoint[i];
  }
  seds->dT = dT;

  seds->Priors.Resize(nclusters);
  seds->Mu.Resize(dim, nclusters);
  seds->Sigma = new Matrix[nclusters];
  for (int i = 0; i < nclusters; i++){
    seds->Sigma[i].Resize(dim,dim);
    seds->Priors(i) = gmm->c_gmm->gauss[i].prior;
    for (int j = 0; j < dim; j++){
      seds->Mu(j,i) = gmm->c_gmm->gauss[i].mean[j];
      for (int k = 0; k < j + 1; k++){
	seds->Sigma[i](k,j) = seds->Sigma[i](j,k) = smat_get_value(gmm->c_gmm->gauss[i].covar, k, j);
      }
    }
  }

  seds->nData = samples.size();
  seds->d = dim / 2;
  seds->K = nclusters;
  seds->Options.cons_penalty = 100;
  seds->Options.perior_opt = true;
  seds->Options.mu_opt = true;
  seds->Options.sigma_x_opt = true;
  seds->Options.max_iter = 600;
  seds->Options.tol_mat_bias = 1e-8;
  seds->Options.tol_stopping = 1e-12;
  seds->Options.objective = 1;

  seds->Optimize();

  delete data;
  delete ddata;
  delete gmm;
}

void process_bagfile(string filename, vector< vector<fvec> > &trajectories, float &dT, string &source_fid, string &target_fid){

    // initialize the bag view
    rosbag::Bag bag;
    bag.open(filename, rosbag::bagmode::Read);
    ROS_INFO("Processing bag: %s", filename.c_str());
    rosbag::View view(bag, rosbag::TopicQuery("seds/trajectories"));

    // process source and target fids
    source_fid = "";
    target_fid = "";
    vector<string> topics;
    topics.push_back("seds/source_fid");
    topics.push_back("seds/target_fid");
    rosbag::View tf_frame_info(bag, rosbag::TopicQuery(topics));

    BOOST_FOREACH(rosbag::MessageInstance const m, tf_frame_info){
      std_msgs::String::ConstPtr s = m.instantiate<std_msgs::String>();

      ROS_INFO("TOPIC %s", m.getTopic().c_str());
      if (m.getTopic().compare("seds/source_fid") == 0){
	source_fid = s->data;
      } else if (m.getTopic().compare("seds/target_fid") == 0){
	target_fid = s->data;
      }

    }

    ROS_INFO("source_fid: %s", source_fid.c_str());
    ROS_INFO("target_fid: %s", target_fid.c_str());

    // compute some statistics of the sample trajectories
    uint32_t maxindx = 0;
    ivec tlengths;
    uint32_t xsize = 0;
    uint32_t dxsize = 0;
    tlengths.push_back(0);
    bool first = true;
    uint32_t nsamples = 0;
    int minsize = 0;
    int maxsize = 0;

    // compute average dT
    double avgdt = 0.0;
    double ct = 0.0;
    double pt = 0.0;

    // some more methods of computing dT
    vector<double> start_times;
    vector<double> end_times;

    // this first loop collects some statistics about the trajectories
    BOOST_FOREACH(rosbag::MessageInstance const m, view){

      // bags must store SedsMessage messages
      seds::SedsMessage::ConstPtr p = m.instantiate<seds::SedsMessage>();

      if (first){ // collect the sizes of the x and dx data
	xsize = p->x.size();
	dxsize = p->dx.size();
	assert(xsize == dxsize); // these must be the same for SEDS

	first = false;
      }

      if (maxindx < (p->index)){ // determine the # of trajectories
	tlengths.push_back(0);
	maxindx = p->index;
      }

      // need to compute dT
      avgdt += p->dt.toSec();

      // figure out the trajectory lengths
      tlengths[maxindx]++;
      nsamples++;
    }

    dT = avgdt / (float) nsamples;
    ROS_INFO("dT: %f avgdt: %f nsamples: %d", dT, avgdt, nsamples);

    // output basic trajectory information
    ROS_INFO("# of Trajectories: %d", maxindx);
    ROS_INFO("x size: %d", xsize);
    ROS_INFO("dx size: %d", dxsize);

    // resize the container for all the trajectory data
    trajectories.resize(tlengths.size());

    for (int i=0;i<tlengths.size();i++){
      trajectories[i].resize(tlengths[i]);
      ROS_INFO("Traj %d has length %d", i, tlengths[i]);
    }

    start_times.assign(tlengths.size(), 1e12);
    end_times.assign(tlengths.size(), 0.0);



    // this second loop fills in the trajectory data
    int i = 0;
    int lastindex = 0;
    ros::Time t;
    int index;

    BOOST_FOREACH(rosbag::MessageInstance const m, view){
      seds::SedsMessage::ConstPtr p = m.instantiate<seds::SedsMessage>();

      index = p->index;
      t = p->t;

      if (lastindex != p->index){
	// start populating new trajectory data
	i = 0;
	lastindex = p->index;
      }

      if (start_times[p->index] > p->t.toSec()){
	start_times[p->index] = p->t.toSec();
      }

      if (end_times[p->index] < p->t.toSec()){
	end_times[p->index] = p->t.toSec();
      }

      // finally we can actually populate the data
      trajectories[p->index][i].resize(xsize + dxsize);
      for (int j=0; j<xsize; j++){
	trajectories[p->index][i][j] = p->x[j];
      }

      for (int k=0; k<dxsize; k++){
	trajectories[p->index][i][xsize + k] = p->dx[k];
      }

      i += 1;
    }

    ROS_INFO("DIFF TIMES:");
    for(int i=0;i<start_times.size();i++){
      ROS_INFO("et : %f st : %f diff : %f len: %d dt : %f", end_times[i], start_times[i], end_times[i] - start_times[i], tlengths[i], (end_times[i] - start_times[i]) / tlengths[i]);
    }

    // normalize trajectory lengths
    minsize = *min_element( tlengths.begin(), tlengths.end() );
    maxsize = *max_element( tlengths.begin(), tlengths.end() );
    ROS_INFO("Min size: %d", minsize);
    ROS_INFO("Max size: %d", maxsize);

    // double check
    for (int i=0;i<tlengths.size();i++){
      ROS_INFO("Traj %d now has length %d", i, trajectories[i].size());
    }
}
