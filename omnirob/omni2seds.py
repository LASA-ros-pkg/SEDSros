#! /usr/bin/env python
"""
Author:  Christophe Paccolat
Program: OMNI2SEDS.PY
Date: Monday, August 15th, 2011
Description: Create a bagfile of SedsMessages for optimization.
"""

import roslib
roslib.load_manifest('seds')
import rospy
import rosbag
import sys
from seds.msg import SedsMessage
import getopt
import glob
import os
import numpy as np
import math
from copy import deepcopy
npa = np.array

def iszero(arr):
    for e in arr:
        if math.fabs(e) > 1e-4:
            return False
    return True

def process_bags(outfilename, inbags):

    outbag = rosbag.Bag(outfilename, "w")
    pm = SedsMessage() # store previous message for dx computation
    #pm.x = [0.0] * 9
    cm = SedsMessage() # current seds message
    #cm.x = [0.0] * 9
    zero = rospy.rostime.Duration(0)

    # open all the trajectory bags
    for (i,bagname) in enumerate(inbags):
        rospy.loginfo("Processing bag %d: %s" %  (i,bagname))

        # need a new listener for each bag -- otherwise TF_OLD_DATA warnings
        bag = rosbag.Bag(bagname)
        first = True # set pm on first step -- compute pm.dx on every other step
	
        # read all the bag messages and process /tf messages
        for topic, msg, t in bag.read_messages():

            if topic == "/joints_fb":	
                # Check for updated values
                cm.t = t
                cm.index = i
                cm.x = npa(msg.position)
#                id = 0
#                for j in msg.jointnumber:
#                    cm.x[j] =  msg.position[id]
#                    id+=1

                # Initial copy
                if first:
                    pm.x = deepcopy(cm.x)
                    pm.index = cm.index
                    pm.t = cm.t
                    first = False
		
                # compute dx
                pm.dx = npa(cm.x) - npa(pm.x)

                # compute dt
                pm.dt = cm.t - pm.t

                # Write out the seds bag if time has changed.
                # Some tf frames change at a much faster rate and
                # so listener reprocesses the same transform
                # lookup even though latest common time remains
                # the same. If we did not check pm.dt != zero,
                # we'd end up with a lot of duplicate entries.

                # We also check whether pm.dx is zero. This choice
                # is the result of a fair amount of
                # experimentation to try to extract the actual
                # motions from a bagfile that probably contains a
                # lot of the robot just sitting there (e.g. before
                # the teaching trajectory actually starts). I
                # experimented a fair bit with the best way to
                # extract the useful subtrajectory out of the bag
                # and this criterion is the best in terms of
                # simplicity and quality.        

                if (pm.dt != zero and not iszero(pm.dx)):
                	rospy.logdebug("Message time: %s" % str(pm.t))
                	outbag.write('seds/trajectories', pm)

                # alt. set t parameter to orginal bag
                # time? would need to process bags in
                # order to avoid time travel
		
                pm.x = deepcopy(cm.x)
                pm.index = cm.index
                pm.t = cm.t

        # end of current bag -- write out last entry with dx = 0      
	cm.dx = npa(cm.x) - npa(pm.x)

        outbag.write('seds/trajectories', cm)

        bag.close()

    rospy.loginfo("Writing bag: %s" % outfilename)
    outbag.close() # very important to close a rosbag after writing

def main():

    #Example usage:
    #rosrun wam2seds.py -b <path_to_wam_bag_files> -o <path_to_seds_bagfile>

    """
    Right now process_bags only uses x,y,z position information.
    """

    # rospy gets first crack at sys.argv
    rospy.myargv(argv=sys.argv)
    (options,args) = getopt.getopt(sys.argv[1:], 'b:o:s:t:', ['bags=','ouptut=','source=','target='])

    rospy.init_node("omni2seds")
    path = rospy.get_param("/omni2seds/source_directory", None)
    outfile = rospy.get_param("/omni2seds/outputfile", None)

    # or specify any of these options on the command line
    for o,a in options:
        if o in ('-b','--bags'):
            path = a
        elif o in ('-o','--output'):
            outfile = a
        elif o in ('-s','--source'):
            source_frameid = a
        elif o in ('-t','--target'):
            target_frameid = a

    if not path or not outfile:
        raise Exception, "You must specify bagfile input and output locations."

    bagfiles = glob.glob(os.path.normpath(path + "/*.bag"))
    bagfiles.sort() # given the default naming these should be sorted by time
    rospy.loginfo("dir: %s out: %s", path, outfile)
    process_bags(outfile, bagfiles)


if __name__ == '__main__':
    main()
