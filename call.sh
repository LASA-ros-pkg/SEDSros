#!/bin/bash

#rosservice call seds_server "/home/stober/workspace/ros/seds/data/test.ml" 0 "model_ml.txt"
#rosservice call seds_server "/home/stober/workspace/ros/seds/data/bag_seds.bag" 1 "/home/stober/workspace/ros/seds/models/model_ts.txt"
#rosservice call seds_server "/home/stober/workspace/ros/bags/seds.bag" 1 "model_pr2.txt"

# turtlesim

rosservice call load_model "/home/stober/workspace/ros/seds/models/model_ts.txt"
read
rosservice call ds_server "[7.0, 7.0]"


# pr2

#rosservice call load_model "/home/stober/workspace/ros/seds/models/model_pr2.txt"
#read
#rosservice call ds_server "[0.26886435852375368, -0.66369020531995493, -0.33106905311020346, -0.026356518273423155, 0.57636504098853847, 0.066325298676144173, 0.81406979321573136]"