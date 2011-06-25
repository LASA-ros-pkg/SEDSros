#! /usr/bin/env python
"""
Author: Jeremy M. Stober
Program: CAT_TRANSFORM.PY
Date: Monday, June 20 2011
Description: Script to display the latest available tf transform in easy to parse format.
"""

import roslib
roslib.load_manifest('tf')
roslib.load_manifest('seds')

import tf
import rospy
import rospy.rostime as rostime
from geometry_msgs.msg import PoseStamped
from seds.srv import DSSrv
import numpy
npa = numpy.array

if __name__ == '__main__':

    source_frameid = 'torso_lift_link'
    target_frameid = 'r_gripper_tool_frame'

    rospy.init_node('ds_driver')
    listener = tf.TransformListener()

    pub = rospy.Publisher('r_cart/command_pose', PoseStamped)

    # wait for the ds server
    rospy.wait_for_service('ds_server')
    ds = rospy.ServiceProxy('ds_server', DSSrv) # the running model

    # waits 10 secs for the source and target frames to become available
    try:
        listener.waitForTransform(source_frame=source_frameid, target_frame=target_frameid,
                                  time=rostime.Time(0), timeout=rospy.Duration(10.0))
    except tf.Exception, error:
        print error

    cmd = PoseStamped()
    cmd.header.frame_id = "/torso_lift_link"
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():

        et = listener.lookupTransform(source_frameid, target_frameid, rostime.Time(0))
        t = listener.getLatestCommonTime(source_frameid, target_frameid)

        rot = list(et[1][:])
        x = list(et[0][:])

        rospy.loginfo("x: %s" % str(x))

        dx = list(ds(x).dx)

        rospy.loginfo("dx: %s" % str(dx))
        newx = list(npa(x) + 25*npa(dx))

        rospy.loginfo("nx: %s" % str(newx))

        cmd.pose.position.x = newx[0]
        cmd.pose.position.y = newx[1]
        cmd.pose.position.z = newx[2]

        # just use the tf pose orientations
        cmd.pose.orientation.x = rot[0]
        cmd.pose.orientation.y = rot[1]
        cmd.pose.orientation.z = rot[2]
        cmd.pose.orientation.w = rot[3]

        pub.publish(cmd)

        rate.sleep()
