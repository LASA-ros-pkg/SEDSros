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

class PR2Driver:

    def __init__(self, vm, feedback, source_frameid, target_frameid, rate, waittf):

        rospy.loginfo("Initialized with vm: %f and feedback: %s on sf: %s and tf: %s" % (vm, str(feedback), source_frameid, target_frameid))

        self.useseds = True
        self.vm = vm
        self.feedback = feedback
        self.source_frameid = source_frameid
        self.target_frameid = target_frameid
        self.model_source_frameid = ""
        self.model_target_frameid = ""

        self.rate = rate

        self.listener = tf.TransformListener()
        self.pub = rospy.Publisher('r_cart/command_pose', PoseStamped)

        # wait for the ds server
        rospy.loginfo('Waiting for ds_node...')
        rospy.wait_for_service('/ds_node/ds_server')
        rospy.loginfo('ds_node found!')

        self.dl = rospy.ServiceProxy('/ds_node/is_loaded', DSLoaded) # check whether the ds_server has a model
        self.ds = rospy.ServiceProxy('/ds_node/ds_server', DSSrv) # the running model
        self.dsparams = rospy.ServiceProxy('/ds_node/params', SedsModel)

        self.startSRV = rospy.Service('/pr2_driver/start', Empty, self.start)
        self.stopSRV = rospy.Service('/pr2_driver/stop', Empty, self.stop)
        self.quitSRV = rospy.Service('/pr2_driver/quit', Empty, self.quit)
        self.vmSRV = rospy.Service('/pr2_driver/setvm', FloatSrv, self.setvm)
        self.rateSRV = rospy.Service('/pr2_driver/setrate', IntSrv, self.setrate)
        self.fbSRV = rospy.Service('/pr2_driver/toggle_feedback', Empty, self.toggleFb)
        self.toggleSRV = rospy.Service('/pr2_driver/toggle_seds', Empty, self.toggleSeds)

        self.zerot = rostime.Time(0)

        # wait for the proper /tf transforms
        if waittf:
            self.wait_for_transform(source_frameid, target_frameid)

        self.cmd = PoseStamped()
        self.cmd.header.frame_id = "/" + source_frameid

        self.running = False
        self.runningCV = threading.Condition()

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

    def toggleSeds(self, ignore):
        self.runningCV.acquire()
        self.useseds = not self.useseds
        self.runningCV.release()
        rospy.loginfo("Toggling seds to %s" % self.useseds)
        return []

    def setvm(self, req):

        self.runningCV.acquire()
        self.vm = req.value
        self.runningCV.release()
        rospy.loginfo("VM set to %f" % self.vm)
        return []

    def setrate(self, req):
        self.runningCV.acquire()
        self.rate = rospy.Rate(req.value)
        self.runningCV.release()
        rospy.loginfo("Rate set to %d" % req.value)
        return []

    def toggleFb(self, ignore):
        self.runningCV.acquire()
        self.feedback = not self.feedback
        self.runningCV.release()
        rospy.loginfo("Toggling feedback to %s" % self.feedback)
        return []

    def quit(self, ignore):
        """
        Call the quit service to quit the pr2_driver.
        """
        self.runningCV.acquire()
        self.running = False
        rospy.core.signal_shutdown("quit pr2_driver")
        self.runningCV.release()
        return []


    def stop(self, ignore):
        """
        Call the stop service to stop the pr2_driver.
        """
        self.runningCV.acquire()

        self.running = False
        rospy.loginfo("pr2_driver stopping!")

        self.runningCV.release()
        return []

    def start(self, ignore):
        """
        Call the start service to start the pr2_driver.
        """

        self.runningCV.acquire()

        res = self.dl()
        if res.loaded:

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

            self.running = True
            rospy.loginfo("pr2_driver starting!")
            # need to send a signal to wake up the main thread?
        else:
            rospy.loginfo("ds_node model is not loaded -- not starting!")

        self.runningCV.notify()
        self.runningCV.release()
        return []

    def spin(self):

        rospy.loginfo("Running!")

        try:

            while not rospy.is_shutdown():

                self.runningCV.acquire()
                if self.running:

                    # if feedback is true then re-intialize x on every loop using tf
                    et = self.listener.lookupTransform(self.model_source_frameid, self.model_target_frameid, rostime.Time(0))

                    if self.feedback:
                        self.x = list(et[0][:])
                    else:
                        self.x = self.newx

                    rospy.logdebug("x: %s" % str(self.x))

                    if self.useseds:
                        self.dx = list(self.ds(self.x).dx)

                        rospy.logdebug("dx: %s" % str(self.dx))
                        self.newx = list(npa(self.x) + self.vm * npa(self.dx))

                        rospy.logdebug("nx: %s" % str(self.newx))
                    else: # just set the endpoint and let JTTeleop take us there
                        self.newx = self.endpoint

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
                    self.rate.sleep()

                else:
                    # wait around until a start service call
                    # check for an interrupt every once and awhile
                    self.runningCV.wait(1.0)

                self.runningCV.release()

        except KeyboardInterrupt:
            rospy.logdebug('keyboard interrupt, shutting down')
            rospy.core.signal_shutdown('keyboard interrupt')

def main():

    # rospy gets first crack at sys.argv
    rospy.myargv(argv=sys.argv)
    (options,args) = getopt.getopt(sys.argv[1:], 'v:fs:t:', ['vm=','feedback','source=','target='])

    rospy.init_node('pr2_driver')

    source_frameid = rospy.get_param("/r_cart/root_name","torso_lift_link")
    target_frameid = rospy.get_param("/r_cart/tip_name","r_gripper_tool_frame")
    vm = rospy.get_param("/pr2_driver/velocity_multiplier", 25.0)
    feedback = rospy.get_param("/pr2_driver/feedback", True)

    for o,a in options:
        if o in ('-v','--vm'):
            vm = float(a)
        elif o in ('-f','--feedback'):
            feedback = True
        elif o in ('-s','--source'):
            source_frameid = a
        elif o in ('-t','--target'):
            target_frameid = a

    driver = PR2Driver(vm, feedback, source_frameid, target_frameid, rospy.Rate(100), False) # start node
    driver.spin()

if __name__ == '__main__':
    main()
