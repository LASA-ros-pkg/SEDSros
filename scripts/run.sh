#!/bin/bash -x

#export ROS_MASTER_URI=http://lasapc7:11311

# setup the ds_node with the current model
rosrun seds ds_node &
#rosrun seds gmr.py &
sleep 2

rosservice call load_model $1
sleep 2

# iteratively calls ds_server on latest tf position and publishes the result to r_cart/command_pose
rosrun seds ds_driver.py --vm 25 --feedback
#rosrun seds ds_driver.py --vm 1.0
