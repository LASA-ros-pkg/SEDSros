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
// 1. Get trajectories from ROS source. (turtlesim?)
// 2. Return model parameters.
// 3. Run GMM style controller with model parameters. (turtlesim?)

bool sedsSRV(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res )
{
  DatasetManager ds = DatasetManager();
  DynamicalSEDS seds = DynamicalSEDS();

  ds.Load("/home/stober/workspace/ros/seds/data/test.ml");

  // turning samples into trajectories is somewhat messy -- borrowed from the original mldemos code in mlprocessing.cpp
  vector<fvec> samples = ds.GetSamples();
  vector<ipair> sequences = ds.GetSequences();
  ivec labels = ds.GetLabels();
  if(!samples.size() || !sequences.size()) return false;
  int dim = samples[0].size();
  int count = options.count;
  int resampleType = options.resampleType;
  int centerType = options.centerType;
  bool zeroEnding = options.zeroEnding;

  // we split the data into trajectories
  vector< vector<fvec> > trajectories;
  ivec trajLabels;
  trajectories.resize(sequences.size());
  trajLabels.resize(sequences.size());
  FOR(i, sequences.size())
    {
      int length = sequences[i].second-sequences[i].first+1;
      trajLabels[i] = ds.GetLabel(sequences[i].first);
      trajectories[i].resize(length);
      FOR(j, length)
        {
	  trajectories[i][j].resize(dim*2);
	  // copy data
	  FOR(d, dim) trajectories[i][j][d] = samples[sequences[i].first + j][d];
        }
    }

  switch(resampleType)
    {
    case 0: // none
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


  if(centerType)
    {
      map<int,int> counts;
      map<int,fvec> centers;
      FOR(i, sequences.size())
        {
	  int index = centerType ? sequences[i].second : sequences[i].first; // start
	  int label = ds.GetLabel(index);
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
	  fvec difference = centers[trajLabels[i]] - trajectories[i][count-1];
	  FOR(j, count) trajectories[i][j] += difference;
        }
    }

  float dT = options.dT; // time span between each data frame
  seds.dT = dT;

  float maxV = -FLT_MAX;

  // we compute the velocity
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
