#! /usr/bin/env python
"""
Author: Jeremy M. Stober
Program: RUN_TF_BAG.PY
Date: Monday, June 20 2011
Description: Replay tf coordinates as control commands.
"""

import roslib
roslib.load_manifest('tf')
import rospy

from tf import TransformerROS
import threading
from tf.msg import tfMessage

from geometry_msgs.msg import PoseStamped
import rospy.rostime as rostime

"""
Supposedly, tf_prefix is supposed to allow for the kind of tf
manipulations we would need to replay a bag file of tf information
into a command topic on a running robot (which is also producing tf
information). I was not able to get tf_prefix to work. Digging around
the Python code for tf led me to the following Listener thread which
seemed easy enough to modify for our purposes (by switching the
subscriber topic to /tf_old).
"""

class TransformListenerThread(threading.Thread):
    def __init__(self, tl):
        threading.Thread.__init__(self)
        self.tl = tl

    def run(self):
        rospy.Subscriber("/tf_old", tfMessage, self.transformlistener_callback)

    def transformlistener_callback(self, data):
        who = data._connection_header.get('callerid', "default_authority")
        for transform in data.transforms:
            self.tl.setTransform(transform, who)

class TransformListener(TransformerROS):

    def __init__(self, *args):
        TransformerROS.__init__(self, *args)
        thr = TransformListenerThread(self)
        thr.setDaemon(True)
        thr.start()
        self.setUsingDedicatedThread(True)


if __name__ == '__main__':

    source_frameid = 'torso_lift_link'
    target_frameid = 'r_gripper_tool_frame'
    pub = rospy.Publisher('r_cart/command_pose', PoseStamped)

    rospy.init_node('run_tf_bag')

    # setting the rate results in a strange error during sleep ("ROS moved backwards in time.")
    #rate = rospy.Rate(10.0)
    listener = TransformListener() # this is our own local version that listens to tf_old

    # waits 10 secs for the source and target frames to become available
    try:

        listener.waitForTransform(source_frame=source_frameid, target_frame=target_frameid,
                                  time=rostime.Time(0), timeout=rospy.Duration(10.0))
    except tf.Exception, error:
        print error

    print "Found transform!"

    cmd = PoseStamped()
    while not rospy.is_shutdown():

        et = listener.lookupTransform(source_frameid, target_frameid, rostime.Time(0))
        t = listener.getLatestCommonTime(source_frameid, target_frameid)

        cmd.header.frame_id = "/torso_lift_link"
        cmd.pose.position.x = et[0][0]
        cmd.pose.position.y = et[0][1]
        cmd.pose.position.z = et[0][2]
        cmd.pose.orientation.x = et[1][0]
        cmd.pose.orientation.y = et[1][1]
        cmd.pose.orientation.z = et[1][2]
        cmd.pose.orientation.w = et[1][3]

        #rospy.loginfo(cmd)

        pub.publish(cmd)

        #rate.sleep()
