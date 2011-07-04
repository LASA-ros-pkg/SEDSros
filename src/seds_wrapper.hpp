#ifndef SEDS_WRAPPER_HPP
#define SEDS_WRAPPER_HPP

#include "ros/ros.h"

#include "boost/foreach.hpp"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "seds/SedsMessage.h"
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


void process_bagfile(string filename, vector< vector<fvec> > &trajectories, float &dT);
void seds_optimize(SEDS *seds, vector< vector<fvec> > trajectories, float dT);

#endif
