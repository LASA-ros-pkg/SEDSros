#ifndef PROCESS_BAGFILE_HPP
#define PROCESS_BAGFILE_HPP

#include "ros/ros.h"

#include "boost/foreach.hpp"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "seds/SedsMessage.h"
#include "mldemos/types.h"
#include "mldemos/mymaths.h"
#include <assert.h>

using namespace std;

void process_bagfile(string filename, vector< vector<fvec> > &trajectories, ivec &labels, float &dT);

#endif
