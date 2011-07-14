#! /usr/bin/env python
"""
Author: Jeremy M. Stober
Program: PR2_DRIVER.PY
Date: Monday, June 20 2011
Description: Publishes ds commands to r_cart/command_pose.
"""

import roslib
roslib.load_manifest('tf')
roslib.load_manifest('seds')
import driver

import tf
import rospy
import rospy.rostime as rostime
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped

from seds.srv import DSSrv
from seds.srv import DSLoaded
from seds.srv import FloatSrv, IntSrv
from seds.srv import SedsModel
from std_srvs.srv import Empty


import numpy
import getopt
import sys
npa = numpy.array

import threading

class PR2Driver(driver.Driver):

    def init_subscriber(self):
        self.listener = tf.TransformListener()

    def init_publisher(self):
        self.pub = rospy.Publisher('r_cart/command_pose', PoseStamped)


    def __init__(self, name, vm, feedback, rate, source_frameid, target_frameid, waittf):

        driver.Driver(name, vm, feedback, rate)

        self.source_frameid = source_frameid
        self.target_frameid = target_frameid
        self.model_source_frameid = ""
        self.model_target_frameid = ""

        # wait for the proper /tf transforms
        if waittf:
            self.wait_for_transform(source_frameid, target_frameid)

        self.cmd = PoseStamped()
        self.cmd.header.frame_id = "/" + source_frameid

    def wait_for_transform(self,sfid, tfid):
        """
        Blocks until a transform is found.
        """

        rospy.loginfo('Waiting for transform...')
        tfound = False
        while not tfound:
            try:
                self.listener.waitForTransform(source_frame=source_frameid, target_frame=target_frameid,time=self.zerot,timeout=rostime.Duration(10))
                tfound = True # no exception
            except tf.Exception, error:
                print error
        rospy.loginfo('Transform found!')

    def init_start(self):

        model = self.dsparams()
        rospy.loginfo("Dim %d" % model.model.dim)
        self.endpoint = npa(model.model.offset)[:model.model.dim/2]
        self.model_source_frameid = model.model.source_fid
        self.model_target_frameid = model.model.target_fid

        rospy.loginfo("Using endpoint %s" % str(self.endpoint))
        rospy.loginfo("Using model sid: %s fid: %s and controller sid: %s fid: %s" % (self.model_source_frameid,
                                                                                      self.model_target_frameid,
                                                                                      self.source_frameid,
                                                                                      self.target_frameid))
        # nothing will work if this is not true!
        assert self.model_target_frameid == self.target_frameid

        # model source is typically an object, controller source is something like torso_lift_link

        # init some variables (in model frame)
        et = self.listener.lookupTransform(self.model_source_frameid, self.model_target_frameid, self.zerot)
        self.x = list(et[0][:])
        self.newx = self.x


    def get_current_position(self):
        # feedback for input into seds
        et = self.listener.lookupTransform(self.model_source_frameid, self.model_target_frameid, rostime.Time(0))
        return et[0][:]

    def publish(self):
        rospy.logdebug("x : %s dx : %s newx : %s" % (str(self.x), str(self.dx), str(self.newx)))

        # need to transform the new position newx into self.source_frameid reference
        model_command = PointStamped()
        model_command.header.frame_id = "/" + self.model_source_frameid

        model_command.point.x = self.newx[0]
        model_command.point.y = self.newx[1]
        model_command.point.z = self.newx[2]

        control_command = self.listener.transformPoint(self.source_frameid, model_command)

        rospy.logdebug("model_command %s control_command %s" % (str(model_command), str(control_command)))

        self.cmd.pose.position.x = control_command.point.x
        self.cmd.pose.position.y = control_command.point.y
        self.cmd.pose.position.z = control_command.point.z

        # just use the last tf pose orientations
        ct = self.listener.lookupTransform(self.source_frameid, self.target_frameid,rostime.Time(0))
        self.rot = list(ct[1][:])
        self.cmd.pose.orientation.x = self.rot[0]
        self.cmd.pose.orientation.y = self.rot[1]
        self.cmd.pose.orientation.z = self.rot[2]
        self.cmd.pose.orientation.w = self.rot[3]

        self.pub.publish(self.cmd)

def main():

    # rospy gets first crack at sys.argv
    rospy.myargv(argv=sys.argv)
    (options,args) = getopt.getopt(sys.argv[1:], 'v:f:s:t:', ['vm=','feedback=','source=','target='])

    rospy.init_node('pr2_driver')

    source_frameid = rospy.get_param("/r_cart/root_name","torso_lift_link")
    target_frameid = rospy.get_param("/r_cart/tip_name","r_gripper_tool_frame")
    vm = rospy.get_param("/pr2_driver/velocity_multiplier", 10.0)
    feedback = rospy.get_param("/pr2_driver/feedback", 'hard')

    for o,a in options:
        if o in ('-v','--vm'):
            vm = float(a)
        elif o in ('-f','--feedback'):
            assert a in ('none','hard','adaptive')
            feedback = a
        elif o in ('-s','--source'):
            source_frameid = a
        elif o in ('-t','--target'):
            target_frameid = a

    driver = PR2Driver("pr2_driver", vm, feedback, 100, source_frameid, target_frameid, False) # start node
    driver.spin()

if __name__ == '__main__':
    main()
