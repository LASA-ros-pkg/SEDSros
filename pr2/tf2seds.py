#! /usr/bin/env python
"""
Author: Jeremy M. Stober
Program: tf2seds.py
Date: Monday, June 20 2011
Description: Create a seds message bag out of a set of bagfiles that record /tf trajectories.
"""

import roslib
roslib.load_manifest('tf')
roslib.load_manifest('seds')
import rospy

import math
import numpy
npa = numpy.array

from tf import TransformerROS, LookupException
from seds.msg import SedsMessage

import rospy.rostime as rostime
import rosbag

# based on Listener in tf package -- modified to make it easy to send in bag data directly
class BagListener(TransformerROS):

    def __init__(self, *args):
        TransformerROS.__init__(self, *args)

    def load(self, data):
        for transform in data.transforms:
            self.setTransform(transform, "tf2seds")

def iszero(arr):
    for e in arr:
        if math.fabs(e) > 1e-10:
            return False
    return True

def process_bags(outfilename, inbags, source_fid, target_fid):

    outbag = rosbag.Bag(outfilename, "w")
    pm = SedsMessage() # store previous message for dx computation
    cm = SedsMessage() # current seds message
    zero = rostime.Duration(0)

    # open all the trajectory bags
    for (i,bagname) in enumerate(inbags):
        rospy.loginfo("Processing bag %d: %s" %  (i,bagname))

        # need a new listener for each bag -- otherwise TF_OLD_DATA warnings
        listener = BagListener()
        bag = rosbag.Bag(bagname)
        first = True # set pm on first step -- compute pm.dx on every other step

        # read all the bag messages and process /tf messages
        for topic, msg, t in bag.read_messages():

            if topic[-2:] == "tf":
                # loads the transforms
                listener.load(msg)

                # computes the coordinates
                try:
                    et = listener.lookupTransform(source_fid, target_fid, rostime.Time(0))
                    t = listener.getLatestCommonTime(source_fid, target_fid)

                    # fill in the seds message
                    cm.x = et[0][:] + et[1][:] # pos (x,y,z) and orient (x,y,z,w)
                    cm.index = i
                    cm.t = t

                    if first:
                        pm.x = cm.x
                        pm.index = cm.index
                        pm.t = cm.t
                        first = False

                    # compute dx
                    pm.dx = npa(cm.x) - npa(pm.x)

                    # compute dt
                    pm.dt = cm.t - pm.t;

                    # write out the seds bag if time has changed
                    # some tf frames change at a much faster rate
                    # and so listener reprocesses the same
                    # transform lookup even though latest common
                    # time remains the same

                    if (pm.dt != zero and not iszero(pm.dx)):
                        print pm.t
                        outbag.write('seds/trajectories', pm)

                        # alt. set t parameter to orginal bag
                        # time? would need to process bags in
                        # order to avoid time travel

                    pm.x = cm.x
                    pm.index = cm.index
                    pm.t = cm.t

                except LookupException, error:
                    # sometimes not enough info is recorded to complete the transform lookup
                    rospy.logdebug("%s %s %s %s" % (error,topic,msg,t))


        # end of current bag -- write out last entry with dx = 0
        cm.dx = npa(cm.x) - npa(pm.x)
        outbag.write('seds/trajectories', cm)

        bag.close()
    outbag.close() # very important to close a rosbag after writing

if __name__ == '__main__':

    source_frameid = 'torso_lift_link'
    target_frameid = 'r_gripper_tool_frame'

    # collected on bosch pr2
    bagfiles = ['/home/stober/workspace/ros/bags/2011-06-15-09-37-51.bag',
                '/home/stober/workspace/ros/bags/2011-06-15-09-38-28.bag',
                '/home/stober/workspace/ros/bags/2011-06-15-09-38-55.bag',
                '/home/stober/workspace/ros/bags/2011-06-15-09-39-16.bag',
                '/home/stober/workspace/ros/bags/2011-06-15-09-39-36.bag',
                '/home/stober/workspace/ros/bags/2011-06-15-09-40-07.bag']

    outfile = '/home/stober/workspace/ros/bags/seds.bag'

    # collected in simulator
    # bagfiles = ['/home/stober/workspace/ros/bags/sim/2011-06-20-12-57-34.bag',
    #             '/home/stober/workspace/ros/bags/sim/2011-06-20-14-24-31.bag',
    #             '/home/stober/workspace/ros/bags/sim/2011-06-20-18-55-16.bag']
    # outfile = '/home/stober/workspace/ros/bags/sim/seds.bag'

    rospy.init_node('tf2seds')

    import pdb
    pdb.set_trace()
    process_bags(outfile, bagfiles, source_frameid,target_frameid)
