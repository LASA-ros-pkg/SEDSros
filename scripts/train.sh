#!/bin/bash -x

# Note: roscore should already be running
# The arguments to this script are:
# $1 : location of bagfiles to be read
# $2 : location of seds bag to write
# $3 : location of model parameters to write

# process a set of bagfiles into a seds bag
rosrun seds tf2seds.py $1 $2

# start the seds_server
rosrun seds seds_node &

sleep 2

# process seds bag using seds_server
rosservice call seds_server $2 1 $3

# learning is finished!