#!/bin/bash

# This is the master script for recording, training, then driving the robot using SEDS-based control.

function launch_silent {
    local NODE=$1
    local FILE=$2
    local OPTS=$3
    if [ -z $OPTS ]; then
		roslaunch $NODE $FILE >>/tmp/seds.out 2>>/tmp/seds.err &
    else
		roslaunch $NODE $FILE $OPTS >>/tmp/seds.out &
#	roslaunch $NODE $FILE $OPTS >>/tmp/seds.out 2>>/tmp/seds.err &
    fi

    local PID=$!

    # Return PID cleanly
    echo "$PID"
}

function record_bag {
    # a simple function that records a bagfile in a specified directory
    cd $1
    echo "Prepare to record demonstration number $2 in directory $1."
    echo "Press enter to begin demonstration. When the demonstration is complete press enter again."



    read
    rosbag record /joints_fb &
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
		opt="ss"
        select opt in $OPTIONS; do
	    	if [ "$opt" = "Quit" ]; then
	  			echo "Quiting..."
	  			exit
			elif [ "$opt" = "Record" ]; then
	    		OMNIPID=$(launch_silent orca_proxy orca_base.launch simulatorip:=128.178.145.171)
				#rosservice call /omnirob/idle
	    		record_bag $ddir $ndemo
	    		let ndemo=ndemo+1
			elif [ "$opt" = "Learn" ]; then
				if [ -z $ddir ]; then
					echo "Gimme the bags ! Where are they ?..."
					exit
	    		fi

				echo "Beginning the training process!"
	    		echo "Processing the demonstration bagfiles..."
	    		rosrun seds omni2seds.py -b $ddir -o /tmp/tmp.bag
				if [ $? -ne 0 ]; then
					echo "Error while running seds wrapper script"
					exit
				if

				# optimization
	    		echo "Learning new model parameters using seds!"
	    		rosservice call /seds/optimize /tmp/tmp.bag
	    		if [ $? -ne 0 ]; then
					echo "Error while running SEDS optimization"
					exit
				fi

	    		# in case you need to restart
	    		rosservice call /seds/save_file $(pwd)/$ddir/model.bag
	    		if [ $? -ne 0 ]; then
					echo "Error while saving model to file"
					exit
				fi

	    		echo "Model is learned and loaded."			elif [ "$opt" = "Run" ]; then
				echo "Running the orca driver"
	  			# Issues when running orca like this...
	  			#OMNIPID=$(launch_silent orca_proxy orca_base.launch simulatorip:=128.178.145.171)
	  			#	    echo "pid=$OMNIPID"
	  			# load model parameters into ds_node
	  			DS_STATE=$(rosservice call /ds_node/is_loaded 2>&1)
	  			DS_LOADED="loaded: True"
	  			if [ "$DS_STATE" != "$DS_LOADED" ]; then
	  				if [ -e /tmp/model.bag ]; then
						echo "Loading model from saved file"
						rosservice call /ds_node/load_file $(pwd)/$ddir/model.bag
						if [ $? -ne 0 ]; then
							echo "Error while loading from file"
							exit
						fi
	  				else
						echo "Loading model from cache:"
						rosservice call /ds_node/load_model
						if [ $? -ne 0 ]; then
							echo "Error while loading from cache"
							exit
						fi
	  				fi
	  			fi

	  			DS_STATE=$(rosservice call /ds_node/is_loaded 2>&1)
	  			if [[ $DS_STATE == *ERROR* ]]; then
	  				echo "Could not load any model, did you learn one ?"
					exit
	  			fi

	  			echo "Move the Omnirob to a test position and press enter!"
	  			read

	  			echo "Activating Omnirob and calling Omnirob driver!"
	  			rosservice call /omnirob_driver/start # starts the driver service

	  			echo "Press enter to stop driving the robot."
	  			read
	  			rosservice call /omnirob_driver/stop
	  			#kill -2 $OMNIPID  # send a SIGINT signal to kill orca_proxy
	  			echo "Done"

			else
	  			echo "Bad option!"
			fi
		done
	done
}

# The script requires a working directory.

ddir=$1

mkdir -p $ddir

# Run the SEDS/omnirob related nodes
SEDSPID=$(launch_silent seds omnirob.launch)

# Run the demo.
run_demo

