#include "ros/ros.h"
#include "std_srvs/Empty.h"

#include "boost/foreach.hpp"
#include "rosbag/bag.h"
#include "rosbag/view.h"

#include "seds/SedsOptimize.h"
#include "seds/SedsMessage.h"
#include "mldemos/dynamicalSEDS.h"
#include "process_mlfile.hpp"

bool sedsSRV(seds::SedsOptimize::Request  &req, seds::SedsOptimize::Response &res )
{
  DynamicalSEDS seds = DynamicalSEDS();
  vector< vector<fvec> > trajectories;
  float dT;
  ivec labels;

  ROS_INFO("Called on filetype: %d", req.filetype);

  if (req.filetype == 0){ // an ml file generated using mldemos

    // process the input file into trajectories for training
    string filename = req.filename;
    process_dataset_manager_file(filename, trajectories, labels, dT);
    seds.dT = dT;

    // train the model
    seds.Train(trajectories, labels);

    // save the model
    seds.seds->saveModel(req.outputfile.c_str());
  } else if (req.filetype == 1){ // a bag file

    // initialize the bag view
    rosbag::Bag bag;
    bag.open(req.filename, rosbag::bagmode::Read);
    rosbag::View view(bag, rosbag::TopicQuery("seds/trajectories"));
    uint32_t maxindx = 0;
    ivec tlengths;
    uint32_t xsize = 0;
    uint32_t dxsize = 0;
    tlengths.push_back(0);
    bool first = true;

    ROS_INFO("Processing bag: %s", req.filename.c_str());

    // This first loop collects some statistics about the trajectories.
    BOOST_FOREACH(rosbag::MessageInstance const m, view){
      seds::SedsMessage::ConstPtr p = m.instantiate<seds::SedsMessage>();

      if (first){ // collect the sizes of the x and dx data
	xsize = p->x.size();
	dxsize = p->dx.size();
      }

      if (maxindx < (p->index)){ // determine the # of trajectories
	tlengths.push_back(0);
	maxindx = p->index;
      }

      // figure out the trajectory lengths
      tlengths[maxindx]++;
    }

    ROS_INFO("# of Trajectories: %d", maxindx);
    ROS_INFO("x size: %d", xsize);
    ROS_INFO("dx size: %d", dxsize);

    // resize the container for all the trajectory data
    trajectories.resize(tlengths.size());

    for (int i=0;i<tlengths.size();i++){
      trajectories[i].resize(tlengths[i]);
      ROS_INFO("Traj %d has length %d", i, tlengths[i]);
    }

    // This second loop fills in the trajectory data.
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
    }

  } else {
    ROS_ERROR("Filetype not supported!");
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
