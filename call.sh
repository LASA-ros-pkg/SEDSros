#!/bin/bash

#rosservice call seds_server "/home/stober/workspace/ros/seds/data/test.ml" 0 "model_ml.txt"
#rosservice call seds_server "/home/stober/workspace/ros/seds/turtle/bag_seds.bag" 1 "model_ts.txt"
rosservice call seds_server "/home/stober/workspace/ros/bags/seds.bag" 1 "model_pr2.txt"