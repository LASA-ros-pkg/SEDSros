#ifndef PROCESS_MLFILE_HPP
#define PROCESS_MLFILE_HPP

#include "ros/ros.h"
#include "mldemos/dynamicalSEDS.h"
#include "mldemos/datasetManager.h"

// taken from mlprocessing.cpp and optsDynamic.ui defaults
static struct optionsDynamic {
  int count;
  int resampleType;
  int centerType;
  bool zeroEnding;
  float dT;
} mlfile_options = {100, 1, 0, true, 0.02}; 

void normalize_trajectories(int count, int dim, int maxV,  vector< vector<fvec> > trajectories);
void compute_velocities(int count, int dim, bool zeroEnding, float dT, float &maxV, vector< vector<fvec> > &trajectories);
void center_trajectories(int centerType, int count, vector<ipair> sequences, ivec labels, ivec trajLabels, vector<fvec> samples, vector< vector<fvec> > &trajectories);
void resample_trajectories(int resampleType, int &count, vector<ipair> sequences, vector< vector<fvec> > &trajectories);
void label_trajectories(vector<ipair> sequences, ivec labels, ivec &trajLabels);
void split_samples(vector<fvec> samples, vector<ipair> sequences, int dim, vector< vector<fvec> >  &trajectories);
bool process_dataset_manager_file(string filename, vector< vector<fvec> > &trajectories, ivec &labels, float &dT);

#endif
