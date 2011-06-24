#!/bin/bash -x

# setup the ds_node with the current model
rosrun seds ds_node &
sleep 2

rosservice call load_model $1
sleep 2

# iteratively calls ds_server on latest tf position and publishes the result to r_cart/command_pose
rosrun seds ds_driver.py
