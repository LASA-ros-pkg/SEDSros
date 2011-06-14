#include "ros/ros.h"
#include "std_srvs/Empty.h"

#include "boost/foreach.hpp"
#include "rosbag/bag.h"
#include "rosbag/view.h"

#include "seds/SedsOptimize.h"
#include "mldemos/dynamicalSEDS.h"
#include "process_mlfile.hpp"
#include "turtlesim/Pose.h"


bool sedsSRV(seds::SedsOptimize::Request  &req, seds::SedsOptimize::Response &res )
{
  DynamicalSEDS seds = DynamicalSEDS();
  vector< vector<fvec> > trajectories;
  float dT;
  ivec labels;

  ROS_INFO("Called on filetype: %d", req.filetype);

  if (req.filetype == 0){ // an ml file generated using mldemos

    // process the input file into trajectories for training
    string filename = req.filenames[0];
    process_dataset_manager_file(filename, trajectories, labels, dT);
    seds.dT = dT;

    // train the model
    seds.Train(trajectories, labels);

    // save the model
    seds.seds->saveModel(req.outputfile.c_str());
  } else if (req.filetype == 1){ // a list of bag files

    ROS_INFO("Filesize: %d", req.filenames.size());

    // # of bagfiles -- every bag file contains one trajectory
    trajectories.resize(req.filenames.size());

    for (int i=0;i<req.filenames.size();i++){

      // initialize the bag view
      rosbag::Bag bag;
      // vector<string> topics;
      // topics.push_back("turtle1/pose"); // TODO - make this a generic parameter
      bag.open(req.filenames[i], rosbag::bagmode::Read);
      rosbag::View view(bag, rosbag::TopicQuery("turtle1/pose"));

      ROS_INFO("Processing bag: %s", req.filenames[i].c_str());
      BOOST_FOREACH(rosbag::MessageInstance const m, view){
      	turtlesim::Pose::ConstPtr p = m.instantiate<turtlesim::Pose>();
	ros::Time t = m.getTime();
	ROS_INFO("t: %f", t.toSec());
      	ROS_INFO("x: %f", p->x);
      }


    }
  }
  else {
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
