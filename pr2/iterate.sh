#!/bin/bash -x

# call the ds_node for new dx coordinates
# add dx to x coordinates
# publish new coordinates to rt_cart


# setup the ds_node with the current model
rosrun seds ds_node &
sleep 2
rosservice call load_model "/home/stober/workspace/ros/seds/models/model_pr2.txt"
sleep 2

for i in {1..100}
do

# get current coordinates
coords=`python cat_tf.py`
dx=`rosservice call ds_server "$coords" | perl -ne 'if (m/^dx: (.*)/) {print $1;}'`

echo $coords
echo $dx

# add dx and x
newcoords=`python add.py "$coords" "$dx"`

# publish new coordinates to r_cart/command_pose
python publish_rcart.py "$newcoords"

done