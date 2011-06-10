#include "ros/ros.h"
#include "std_srvs/Empty.h"

#include "seds/SedsOptimize.h"
#include "mldemos/dynamicalSEDS.h"
#include "mldemos/datasetManager.h"

// taken from mlprocessing.cpp and optsDynamic.ui defaults
struct optionsDynamic {
  int count;
  int resampleType;
  int centerType;
  bool zeroEnding;
  float dT;
} options = {100, 1, 0, true, 0.02}; 

// Not actually serving anything. Just running seds against test files for now.

// TODO:
// 1a. Refactor the code that creates the trajectories.
// 1b. How to handle the processing options?
// 1. Get trajectories from ROS source. (turtlesim?)
// 2. Return model parameters.
// 3. Run GMM style controller with model parameters. (turtlesim?)

// NOTES:
// 1. dT is constant for all the trajectory data.
// 2. trajectory sizes are interpolated in order to make them the same length.
// 3. the trajectory length is given by the count parameter.
// 4. in datasetManger, individual trajectories are delimited by the indices in sequences. each is a pair of start,end indices.

void normalize_trajectories(int count, int dim, int maxV,  vector< vector<fvec> > trajectories)
{
  // we normalize the velocities as the variance of the data
  fvec mean, sigma;
  mean.resize(dim,0);
  int cnt = 0;
  sigma.resize(dim,0);
  FOR(i, trajectories.size())
    {
      FOR(j, count)
        {
	  mean += trajectories[i][j];
	  cnt++;
        }
    }
  mean /= cnt;
  FOR(i, trajectories.size())
    {
      FOR(j, count)
        {
	  fvec diff = (mean - trajectories[i][j]);
	  FOR(d,dim) sigma[d] += diff[d]*diff[d];
        }
    }
  sigma /= cnt;

  FOR(i, trajectories.size())
    {
      FOR(j, count)
        {
	  FOR(d, dim)
            {
	      trajectories[i][j][dim + d] /= maxV;
	      //trajectories[i][j][dim + d] /= sqrt(sigma[d]);
            }
        }
    }
}

void compute_velocities(int count, int dim, bool zeroEnding, float dT, float &maxV, vector< vector<fvec> > &trajectories)
{

  // we compute the velocity - zeroEnding sets the final velocity to zero
  FOR(i, trajectories.size())
    {
      FOR(j, count-1)
        {
	  FOR(d, dim)
            {
	      float velocity = (trajectories[i][j+1][d] - trajectories[i][j][d]) / dT;
	      trajectories[i][j][dim + d] = velocity;
	      if(velocity > maxV) maxV = velocity;
            }
        }
      if(!zeroEnding)
        {
	  FOR(d, dim)
            {
	      trajectories[i][count-1][dim + d] = trajectories[i][count-2][dim + d];
            }
        }
    }
}

void center_trajectories(int centerType, int count, vector<ipair> sequences, ivec labels, ivec trajLabels, vector<fvec> samples, vector< vector<fvec> > &trajectories)
{

  // TODO: test whether the center type flags work properly (i'm not sure this will properly switch between start/end point centering).

  // aligns the endpoints or startpoints of all the trajectories
  if(centerType)
    {
      map<int,int> counts;
      map<int,fvec> centers;
      FOR(i, sequences.size())
        {
	  int index = centerType ? sequences[i].second : sequences[i].first; // start
	  int label = labels[index];
	  if(!centers.count(label))
            {
	      fvec center;
	      center.resize(2,0);
	      centers[label] = center;
	      counts[label] = 0;
            }
	  centers[label] += samples[index];
	  counts[label]++;
        }
      for(map<int,int>::iterator p = counts.begin(); p!=counts.end(); ++p)
        {
	  int label = p->first;
	  centers[label] /= p->second;
        }
      FOR(i, trajectories.size())
        {
	  fvec difference = centers[ trajLabels[i] ] - trajectories[i][count-1];
	  FOR(j, count) trajectories[i][j] += difference;
        }
    }
}

void resample_trajectories(int resampleType, int &count, vector<ipair> sequences, vector< vector<fvec> > &trajectories){

  // case 0: make all the trajectories as short as the shortest trajectory.
  // case 1: interpolate to make all the trajectories count long.

  switch(resampleType)
    {
    case 0:
      {
        FOR(i,sequences.size())
	  {
            int cnt = sequences[i].second-sequences[i].first+1;
            if(count > cnt) count = cnt;
	  }
        FOR(i, trajectories.size())
	  {
            while(trajectories[i].size() > count) trajectories[i].pop_back();
	  }
      }
      break;
    case 1: // uniform
      {
        FOR(i, trajectories.size())
	  {
            vector<fvec> trajectory = trajectories[i];
            trajectories[i] = interpolate(trajectory, count);
	  }
      }
      break;
    }
}

void label_trajectories(vector<ipair> sequences, ivec labels, ivec &trajLabels)
{
  trajLabels.resize(sequences.size());
  FOR(i, sequences.size())
    {
      trajLabels[i] = labels[sequences[i].first];
    }
}


void split_samples(vector<fvec> samples, vector<ipair> sequences, int dim, vector< vector<fvec> >  &trajectories)
{
  // helper function that splits samples into separate trajectories based on the sequence indices

  trajectories.resize(sequences.size());
  FOR(i, sequences.size())
    {
      int length = sequences[i].second-sequences[i].first+1;
      trajectories[i].resize(length);
      FOR(j, length)
        {
	  trajectories[i][j].resize(dim*2);
	  // copy data
	  FOR(d, dim) trajectories[i][j][d] = samples[sequences[i].first + j][d];
        }
    }
}

bool process_dataset_manager_file(string filename, vector< vector<fvec> > &trajectories, ivec &labels, float &dT)
{

  // turning samples into trajectories is somewhat messy -- borrowed
  // from the original mldemos code in mlprocessing.cpp

  DatasetManager ds = DatasetManager();
  ds.Load(filename.c_str());

  vector<fvec> samples = ds.GetSamples();
  vector<ipair> sequences = ds.GetSequences();
  ivec trajLabels;
  labels = ds.GetLabels();

  if(!samples.size() || !sequences.size()) return false;
  int dim = samples[0].size();
  int count = options.count;
  int resampleType = options.resampleType;
  int centerType = options.centerType;
  bool zeroEnding = options.zeroEnding;
  dT = options.dT; // time span between each data frame
  float maxV = -FLT_MAX;

  // split the data into trajectories
  split_samples(samples, sequences, dim, trajectories);

  // label the trajectories
  label_trajectories(sequences, labels, trajLabels);

  // resample the trajectories
  resample_trajectories(resampleType, count, sequences, trajectories);

  // center the endpoints, startpoints, or none
  center_trajectories(centerType, count, sequences, labels, trajLabels, samples, trajectories);

  // compute trajectory velocities
  compute_velocities(count, dim, zeroEnding, dT, maxV, trajectories);

  // normalize trajectories
  normalize_trajectories(count, dim, maxV, trajectories);

  return true;
}

bool sedsSRV(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res )
{
  DynamicalSEDS seds = DynamicalSEDS();
  vector< vector<fvec> > trajectories;
  float dT;
  ivec labels;

  string filename = "/home/stober/workspace/ros/seds/data/test.ml";
  process_dataset_manager_file(filename, trajectories, labels, dT);

  seds.dT = dT;
  seds.Train(trajectories, labels);

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
