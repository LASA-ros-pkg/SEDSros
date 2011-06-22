#!/bin/bash -x

# call the ds_node for new dx coordinates
# add dx to x coordinates
# publish new coordinates to rt_cart


# setup the ds_node with the current model
rosrun seds ds_node &
sleep 2
rosservice call load_model "/home/stober/workspace/ros/seds/models/model_pr2_xyz.txt"
sleep 2

# iteratively calls ds_server on latest tf position and publishes the result to r_cart/command_pose
python ds_driver.py
