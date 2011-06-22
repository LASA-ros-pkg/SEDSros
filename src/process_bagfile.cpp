#include "process_bagfile.hpp"

void process_bagfile(string filename, vector< vector<fvec> > &trajectories, ivec &labels, float &dT){

    // initialize the bag view
    rosbag::Bag bag;
    bag.open(filename, rosbag::bagmode::Read);
    ROS_INFO("Processing bag: %s", filename.c_str());
    rosbag::View view(bag, rosbag::TopicQuery("seds/trajectories"));

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

    // this second loop fills in the trajectory data
    int i = 0;
    int lastindex = 0;

    BOOST_FOREACH(rosbag::MessageInstance const m, view){
      seds::SedsMessage::ConstPtr p = m.instantiate<seds::SedsMessage>();

      if (lastindex != p->index){
	// start populating new trajectory data
	i = 0;
	lastindex = p->index;
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

    // normalize trajectory lengths
    minsize = *min_element( tlengths.begin(), tlengths.end() );
    maxsize = *max_element( tlengths.begin(), tlengths.end() );
    ROS_INFO("Min size: %d", minsize);
    ROS_INFO("Max size: %d", maxsize);

    // Do we interpolate or truncate? Not sure...
    // for (int i=0;i<tlengths.size();i++){
    //   while(trajectories[i].size() > minsize) trajectories[i].pop_back();
    // }
    for (int i=0;i<tlengths.size();i++){
      vector<fvec> trajectory = trajectories[i];
      trajectories[i] = interpolate(trajectory, maxsize);
    }

    // double check
    for (int i=0;i<tlengths.size();i++){
      ROS_INFO("Traj %d now has length %d", i, trajectories[i].size());
    }

    // output dummy labels
    nsamples = minsize * tlengths.size();
    labels.resize(nsamples);
    labels.assign(nsamples,1);

    // set dT
}
