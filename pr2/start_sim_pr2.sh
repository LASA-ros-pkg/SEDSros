#!/bin/bash

echo "Launching ROSCORE"

roscore &

sleep 5

echo "Launching PR2 Simulator"

roslaunch pr2_gazebo pr2_empty_world.launch &

roslaunch 

sleep 30

echo "Setting up Cartesian controller"

roslaunch seds jtteleop.launch

