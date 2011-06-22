#!/bin/bash

echo "Launching ROSCORE"

roscore &

sleep 5

echo "Launching PR2 Simulator"

roslaunch pr2_gazebo pr2_empty_world.launch &

sleep 30

echo "Setting up Cartesian controller"

roslaunch teleop_arms jtteleop.launch 

sleep 10

#echo " Running Teleop"

#rosrun teleop_arms teleop_pr2_arms_keyboard