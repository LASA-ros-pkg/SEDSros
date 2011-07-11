#ifndef SEDS_WRAPPER_HPP
#define SEDS_WRAPPER_HPP

#include "ros/ros.h"

#include "boost/foreach.hpp"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "std_msgs/String.h"
#include "seds/SedsMessage.h"
#include "seds/ModelParameters.h"
#include "SEDS.h"
#include "fgmm/fgmm++.hpp"
#include <assert.h>
#include <vector>
#include <cmath>

using namespace std;

// some useful typedefs
typedef vector<double> dvec;
typedef vector<float> fvec;
typedef vector<unsigned int> uvec;
typedef vector<int> ivec;
typedef vector<bool> bvec;


void process_bagfile(string filename, vector< vector<fvec> > &trajectories, float &dT, string &source_fid, string &target_fid);
void seds_optimize(SEDS *seds_obj, vector< vector<fvec> > trajectories, float dT);
void populate_model_msg(SEDS *seds_obj, string source_fid, string target_fid, seds::ModelParameters &model);

#endif
