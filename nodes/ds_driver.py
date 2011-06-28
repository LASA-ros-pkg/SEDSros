#! /usr/bin/env python
"""
Author: Jeremy M. Stober
Program: DS_DRIVER.PY
Date: Monday, June 20 2011
Description: Publishes ds commands to r_cart/command_pose.
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
import getopt
import sys
npa = numpy.array

def init(vm, feedback):

    rospy.loginfo("Called with vm: %f and feedback: %s" % (vm, str(feedback)))

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

    # init some variables
    et = listener.lookupTransform(source_frameid, target_frameid, rostime.Time(0))
    x = list(et[0][:])
    rot = list(et[1][:])
    newx = x

    while not rospy.is_shutdown():

        # t = listener.getLatestCommonTime(source_frameid, target_frameid)

        # if feedback is true then re-intialize x,rot on every loop using tf
        if feedback:
            et = listener.lookupTransform(source_frameid, target_frameid, rostime.Time(0))
            rot = list(et[1][:])
            x = list(et[0][:])
        else:
            x = newx

        rospy.loginfo("x: %s" % str(x))

        dx = list(ds(x).dx)

        rospy.loginfo("dx: %s" % str(dx))
        newx = list(npa(x) + vm * npa(dx))

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
        first = False

def main():

    # rospy gets first crack at sys.argv
    rospy.myargv(argv=sys.argv)
    (options,args) = getopt.getopt(sys.argv[1:], 'v:f', ['vm=','feedback'])

    vm = 1.0
    feedback = False
    for o,a in options:
        if o in ('-v','--vm'):
            vm = float(a)
        elif o in ('-f','--feedback'):
            feedback = True

    init(vm,feedback) # start node

if __name__ == '__main__':
    main()
