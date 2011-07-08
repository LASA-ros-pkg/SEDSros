#! /usr/bin/env python
"""
Author: Jeremy M. Stober
Program: CAT_TF.PY
Date: Monday, June 20 2011
Description: Script to display the latest available tf transform in easy to parse format.
"""

import roslib
roslib.load_manifest('tf')
import tf
import rospy
import rospy.rostime as rostime
import sys

if __name__ == '__main__':

    source_frameid = sys.argv[1]
    target_frameid = sys.argv[2]

    rospy.init_node('cat_tf')
    listener = tf.TransformListener()

    # waits 10 secs for the source and target frames to become available
    try:

        listener.waitForTransform(source_frame=source_frameid, target_frame=target_frameid,
                                  time=rostime.Time(0), timeout=rospy.Duration(10.0))
    except tf.Exception, error:
        print error


    et = listener.lookupTransform(source_frameid, target_frameid, rostime.Time(0))
    t = listener.getLatestCommonTime(source_frameid, target_frameid)

    print t, et
