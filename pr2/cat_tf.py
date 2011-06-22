#! /usr/bin/env python
"""
Author: Jeremy M. Stober
Program: CAT_TRANSFORM.PY
Date: Monday, June 20 2011
Description: Script to display the latest available tf transform in easy to parse format.
"""

import roslib
roslib.load_manifest('tf')
import tf
import rospy
import rospy.rostime as rostime

if __name__ == '__main__':

    source_frameid = 'torso_lift_link'
    target_frameid = 'r_gripper_tool_frame'

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

    print "[%s, %s, %s, %s, %s, %s, %s]" % (et[0][0],et[0][1],et[0][2],et[1][0],et[1][1],et[1][2],et[1][3])
