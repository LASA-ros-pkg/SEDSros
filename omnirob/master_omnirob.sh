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
	echo "Please select one of the following options."
	select opt in $OPTIONS; do
	    if [ "$opt" = "Quit" ]; then
	  echo "Quiting..."
	  exit
	elif [ "$opt" = "Record" ]; then
	    #rosservice call /omnirob/idle
	    record_bag $ddir $ndemo
	    let ndemo=ndemo+1
	elif [ "$opt" = "Learn" ]; then

	    echo "Beginning the training process!"
	    echo "Processing the demonstration bagfiles..."

	    rosrun seds omni2seds.py -b $ddir -o /tmp/tmp.bag

	    echo "Learning new model parameters using seds!"

            # optimization
	    rosservice call /seds/optimize /tmp/tmp.bag

            # load model parameters into ds_node
	    rosservice call /ds_node/load_model

	    # in case you need to restart
	    rosservice call /seds/save_model /tmp/model.bag

	    echo "Model is learned and loaded."
	  break
	elif [ "$opt" = "Run" ]; then

	  #rosservice call /omnirob/idle
	  echo "Move the Omnirob to a test position and press enter!"
	  read

	  echo "Activating Omnirob and calling Omnirob driver!"
	  #rosservice call /omnirob/active # put wam under active control
	  rosservice call /omnirob_driver/start # starts the driver service

	  echo "Press enter to stop driving the robot."
	  read
	  rosservice call /omnirob_driver/stop
	  #rosservice call /omnirob/idle

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

launch_silent seds omnirob.launch &

# Run the demo.
run_demo

