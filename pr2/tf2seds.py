#! /usr/bin/env python
"""
Author: Jeremy M. Stober
Program: TF2SEDS.py
Date: Monday, June 20 2011
Description: Create a seds message bag out of a set of bagfiles that record /tf trajectories.
"""

import roslib
roslib.load_manifest('tf')
roslib.load_manifest('seds')
import rospy

import math
import numpy
import getopt
npa = numpy.array

from tf import TransformerROS, LookupException, ConnectivityException
from tf.transformations import euler_from_quaternion
import tf
from seds.msg import SedsMessage
from std_msgs.msg import String

import rospy.rostime as rostime
import rosbag

import sys
import os
import glob

# based on Listener in tf package -- modified to make it easy to send in bag data directly
class BagListener(TransformerROS):

    def __init__(self, *args):
        TransformerROS.__init__(self, *args)

    def load(self, data):
        for transform in data.transforms:
            self.setTransform(transform, "tf2seds")

def iszero(arr):
    #return False
    for e in arr:
        if math.fabs(e) > 1e-5:
            return False
    return True

def process_bags(outfilename, inbags, source_fid, target_fid):

    outbag = rosbag.Bag(outfilename, "w")
    pm = SedsMessage() # store previous message for dx computation
    cm = SedsMessage() # current seds message
    zero = rostime.Duration(0)


    s1 = String()
    s1.data = source_fid
    outbag.write('seds/source_fid', s1)
    rospy.loginfo("Writing seds/source_fid " + s1.data)

    s2 = String()
    s2.data = target_fid
    outbag.write('seds/target_fid', s2)
    rospy.loginfo("Writing seds/target_fid " + s2.data)

    # open all the trajectory bags
    for (i,bagname) in enumerate(inbags):
        rospy.loginfo("Processing bag %d: %s" %  (i,bagname))

        # need a new listener for each bag -- otherwise TF_OLD_DATA warnings
        listener = BagListener()
        bag = rosbag.Bag(bagname)
        first = True # set pm on first step -- compute pm.dx on every other step

        tf_cnt=0
        tf_match_cnt=0
        nonzero_cnt=0

        # read all the bag messages and process /tf messages
        for topic, msg, t in bag.read_messages():

            #only process /tf messages
            if topic[-2:] == "tf":# and (cfid == target_fid or cfid == source_fid):
                # loads the transforms
                listener.load(msg)
                tf_cnt+=1
                try:
                    #Can we transform between these?
                    #et = listener.lookupTransform(source_fid, target_fid, rostime.Time(0))
                    #since source and target may be at very different rates, we need to time travel
                    #so get times with respect to a constant (the torso)
                    unmoving_fid="torso_lift_link"
                    source_time=listener.getLatestCommonTime(source_fid,unmoving_fid)
                    target_time=listener.getLatestCommonTime(target_fid,unmoving_fid)
                    et = listener.lookupTransformFull(source_fid,source_time,target_fid,target_time,unmoving_fid)
                    tf_match_cnt+=1
                    # get the current x
                    cm.x = et[0][:] + euler_from_quaternion(et[1][:])
                    cm.index = i
                    cm.t = max(source_time,target_time)

                    if first:
                        pm.x = cm.x
                        pm.index = cm.index
                        pm.t = cm.t
                        first = False
                        
                    # compute dx
                    pm.dx = npa(cm.x) - npa(pm.x)

                    # compute dt
                    pm.dt = cm.t - pm.t;
                    
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
                                  
                    if (pm.dt != zero) and not iszero(pm.dx):
                        #rospy.loginfo("Message time: %s" % str(pm.t))
                        outbag.write('seds/trajectories', pm)
                        nonzero_cnt+=1
                        
                        # alt. set t parameter to orginal bag
                        # time? would need to process bags in
                        # order to avoid time travel
                        
                        pm.x = cm.x
                        #pm.index = cm.index
                        pm.t = cm.t
                    

                except LookupException, error:
                    # sometimes not enough info is recorded to complete the transform lookup
                    rospy.logdebug("%s %s %s %s" % (error,topic,msg,t))
                    #rospy.loginfo("LookupException %s" % error)
                except ConnectivityException, error:
                    # sometimes the perceptual information drops out
                    rospy.logdebug("%s %s %s %s" % (error,topic,msg,t))
                    #rospy.loginfo("ConnectivityException %s" % error)
                except tf.Exception, error:
                    rospy.logdebug("%s %s %s %s" % (error,topic,msg,t))
                    #rospy.loginfo("Other Exception %s" % error)

        # end of current bag -- write out last entry with dx = 0
        cm.dx = npa(cm.x) - npa(pm.x)
        outbag.write('seds/trajectories', cm)
        #print "Found " + str(tf_cnt) + "tfs, " + str(tf_match_cnt) + " match, and " + str(nonzero_cnt) + " are nonzero."
        rospy.loginfo("Processed %s, found transforms for %s and %s are nonzero" % (str(tf_cnt), str(tf_match_cnt), str(nonzero_cnt)))
        bag.close()

    rospy.loginfo("Writing bag: %s" % outfilename)
    outbag.close() # very important to close a rosbag after writing

def main():

    #Example usage:
    #rosrun tf2seds.py -b <path_to_tf_bag_files> -o <path_to_seds_bagfile>

    """
    Rotations may cause a problem for SEDS optimization. The
    quaternion representation requires that the magnitude be
    normalized, but the SEDS optimization process does not include
    this constraint. If the magnitude differs from 1.00 then the planned
    motion will not succeed. An Euler representation is also possible,
    but this has not (to my knowledge) been exposed to the Python tf
    API. I'm not sure if there are also constraints for that choice of
    rotation representation.

    So right now process_bags only uses x,y,z position information.
    """


    # rospy gets first crack at sys.argv
    rospy.myargv(argv=sys.argv)
    (options,args) = getopt.getopt(sys.argv[1:], 'b:o:s:t:', ['bags=','ouptut=','source=','target='])

    rospy.init_node("tf2seds")

    # should be in tf2seds namespace
    source_frameid = rospy.get_param("/tf2seds/source_frameid","torso_lift_link")
    target_frameid = rospy.get_param("/tf2seds/target_frameid","r_gripper_tool_frame")
    path = rospy.get_param("/tf2seds/source_directory", None)
    outfile = rospy.get_param("/tf2seds/outputfile", None)

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

    rospy.loginfo("dir: %s out: %s sf: %s tf: %s", path, outfile, source_frameid, target_frameid)


    process_bags(outfile, bagfiles, source_frameid,target_frameid)


if __name__ == '__main__':
    main()
