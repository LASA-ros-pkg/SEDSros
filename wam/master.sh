#!/bin/bash -x

# This is the master script for recording, training, then driving the robot using SEDS-based control.

function launch_silent {
    roslaunch $1 $2 >> /tmp/seds.out 2>> /tmp/seds.err
}

function record_bag {
    # a simple function that records a bagfile in a specified directory
    cd $1
    echo "Prepare to record demonstration number $2 in directory $1."
    echo "Press enter to begin demonstration. When the demonstration is complete press enter again."

    read
    rosbag record -a &
    ID=$!

    read
    kill -2 $ID # send a SIGINT signal to kill the recording process

    # return to original directory
    cd -
}

function run_demo {
    ndemo=1
    OPTIONS="Record Learn Run Quit"
    while [ 1 ]; do

      #clear
      echo "Please select one of the following options."
      select opt in $OPTIONS; do
	if [ "$opt" = "Quit" ]; then
	  echo "Quiting..."
	  exit
	elif [ "$opt" = "Record" ]; then
	  record_bag $ddir $ndemo
	  let ndemo=ndemo+1
	elif [ "$opt" = "Learn" ]; then

	    echo "Beginning the training process!"
	    echo "Processing the demonstration bagfiles..."

	    rosrun seds wam2seds.py -b $ddir -o /tmp/tmp.bag

	    echo "Learning new model parameters using seds!"

            # optimization
	    rosservice call /seds/optimize /tmp/tmp.bag

            # load model parameters into ds_node
	    rosservice call /ds_node/load_model

	    echo "Model is learned and loaded."
	  break
	elif [ "$opt" = "Run" ]; then
	    echo "Not implemented yet!"
	else
	  #clear
	  echo "Bad option!"
	fi
      done
    done
}

# The script requires a working directory.
ddir=$1

mkdir -p $ddir

launch_silent seds wam.launch &

# move to the start position
echo "Press enter to move the wam to the start position!"

read
rosservice call /wam/moveToPos "{pos: {radians: [0.031373526540086058, 0.7948185344899199, -0.049580406514594152, 2.2492436302447012, 0.32928795377115966, 0.055339793705030269, -0.88494097071019928]}}"

echo "Press enter to turn on passive mode!"
read 
rosservice call /wam/switchMode 0
rosservice call /wam/active_passive "[0, 0, 0, 0, 0, 0, 0]"

# Run the demo.
run_demo

