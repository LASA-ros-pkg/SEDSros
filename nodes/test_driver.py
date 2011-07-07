#! /usr/bin/env python
"""
Author: Jeremy M. Stober
Program: TEST_DRIVER.PY
Date: Thursday, June 23 2011
Description: Test the current DS server by recording sample trajectories into a bagfile for analysis.
"""
import os, sys
import roslib
roslib.load_manifest('seds')
import rospy
import rospy.rostime as rostime

from std_srvs.srv import Empty
from seds.srv import DSSrv
from seds.msg import SedsMessage

import rosbag
import numpy
npa = numpy.array

if __name__ == '__main__':

    rospy.init_node('test_ds')
    rospy.myargv(argv=sys.argv)

    rospy.wait_for_service("/ds_node/load_model")
    lm = rospy.ServiceProxy("/ds_node/load_model", Empty)

    rospy.wait_for_service('/ds_node/ds_server')
    ds = rospy.ServiceProxy('/ds_node/ds_server', DSSrv)

    # arg 1 is the model
    # arg 2 is the outbag filename

    # load the model we want to test
    # lm(sys.argv[1])
    lm()

    outbag = rosbag.Bag(sys.argv[1], 'w')
    sm = SedsMessage()

    # for bagfiles collected on 6/15
    starts = [[0.268864,-0.663690,-0.331069],
              [0.156349,-0.542263,0.355905],
              [0.183113,0.272913,-0.263200],
              [0.061941,-0.539489,-0.842995],
              [-0.006358,-1.179837,0.026429],
              [0.215756,-0.564982,-0.329221]]

    # end positions for reference
    ends = [[0.524915,-0.149382,-0.178386],
            [0.511823,-0.299487,-0.089839],
            [0.478417,-0.222597,-0.099429],
            [0.532435,-0.075026,-0.247842],
            [0.531750,-0.071448,-0.256943],
            [0.529323,-0.077282,-0.250892]]

    for (i,s) in enumerate(starts):
        x = s
        px = None

        cnt = 0
        while x != px and cnt < 1000:

            # compute the next step
            dx = list(ds(x).dx)

            # record x,dx in SedsMessage
            sm.x = x
            sm.dx = dx
            sm.t = rospy.Time(0)
            sm.dt = rospy.Duration(0.2)
            sm.index = i

            outbag.write('seds/trajectories', sm)
            rospy.logdebug(x)

            x = list(npa(x) + npa(dx))
            cnt += 1


    outbag.close()
