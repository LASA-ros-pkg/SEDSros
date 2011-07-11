#!/bin/bash -x

rosrun seds ds_node &

sleep 2

# Arguments
# $1 : seds bag to write

rosrun seds test_driver.py $1
