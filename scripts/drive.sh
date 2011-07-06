#!/bin/bash -x

#export ROS_MASTER_URI=http://lasapc7:11311

# setup the ds_node with the current model
rosrun seds ds_node &
#rosrun seds gmr.py &
sleep 2

rosservice call /ds_node/load_file $1
sleep 2

# iteratively calls ds_server on latest tf position and publishes the result to r_cart/command_pose
rosrun seds pr2_driver.py --vm 25 --feedback &

rosservice call seds /pr2_driver/start
