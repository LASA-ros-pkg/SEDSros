#! /usr/bin/env python
"""
Author: Jeremy M. Stober
Program: WAM_DRIVER.PY
Date: Monday, June 20 2011
Description: Publishes ds commands to /wam/cartesian_command.
"""

import roslib
roslib.load_manifest('seds')
roslib.load_manifest('wam_msgs')

import tf
import rospy
import rospy.rostime as rostime

from wam_msgs.msg import CartesianCoordinates
from seds.srv import DSSrv
from seds.srv import DSLoaded
from seds.srv import FloatSrv, IntSrv, StringSrv
from seds.srv import SedsModel
from std_srvs.srv import Empty

import numpy
import numpy.linalg as la
import getopt
import sys
npa = numpy.array

import threading

class WAMDriver:

    def __init__(self, vm, feedback, rate):

        rospy.loginfo("Initialized with vm: %f and feedback: %s" % (vm, str(feedback)))

        self.useseds = True
        self.vm = vm
        self.feedback = feedback
        self.rateInt = rate
        self.rate = rospy.Rate(rate)

        self.pub = rospy.Publisher('/wam/cartesian_command', CartesianCoordinates)

        # wait for the ds server
        rospy.loginfo('Waiting for ds_node...')
        rospy.wait_for_service('/ds_node/ds_server')
        rospy.loginfo('ds_node found!')

        self.dl = rospy.ServiceProxy('/ds_node/is_loaded', DSLoaded) # check whether the ds_server has a model
        self.ds = rospy.ServiceProxy('/ds_node/ds_server', DSSrv) # the running model
        self.dsparams = rospy.ServiceProxy('/ds_node/params', SedsModel)

        self.startSRV = rospy.Service('/wam_driver/start', Empty, self.start)
        self.stopSRV = rospy.Service('/wam_driver/stop', Empty, self.stop)
        self.quitSRV = rospy.Service('/wam_driver/quit', Empty, self.quit)
        self.vmSRV = rospy.Service('/wam_driver/setvm', FloatSrv, self.setvm)
        self.cmptvmSRV = rospy.Service('/wam_driver/computevm', Empty, self.computevm)
        self.rateSRV = rospy.Service('/wam_driver/setrate', IntSrv, self.setrate)
        self.fbSRV = rospy.Service('/wam_driver/change_feedback', StringSrv, self.change_feedback)
        self.toggleSRV = rospy.Service('/wam_driver/toggle_seds', Empty, self.toggleSeds)
        self.stepSRV = rospy.Service('/wam_driver/step', Empty, self.stepsrv)
        self.thresholdSRV = rospy.Service("/wam_driver/set_threshold", FloatSrv, self.set_threshold)

        self.zerot = rostime.Time(0)


        self.cmd = CartesianCoordinates()
        self.current_pose = CartesianCoordinates()

        self.adaptive_threshold = 0.5 # 50 cm

        self.running = False
        self.runningCV = threading.Condition()

        self.sub = rospy.Subscriber('/wam/cartesian_coordinates', CartesianCoordinates, self.cc_callback)


    def cc_callback(self, data):
        self.runningCV.acquire()
        self.current_pose = data
        self.runningCV.release()

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
        self.rateInt = req.value
        self.rate = rospy.Rate(req.value)
        self.runningCV.release()
        rospy.loginfo("Rate set to %d" % req.value)
        return []

    def change_feedback(self, rec):
        self.runningCV.acquire()
        assert str(rec.value) in ('hard','none','adaptive')
        self.feedback = str(rec.value)
        self.runningCV.release()
        rospy.loginfo("Toggling feedback to %s" % self.feedback)
        return []

    def quit(self, ignore):
        """
        Call the quit service to quit the wam_driver.
        """
        self.runningCV.acquire()
        self.running = False
        rospy.core.signal_shutdown("quit wam_driver")
        self.runningCV.release()
        return []


    def stop(self, ignore):
        """
        Call the stop service to stop the wam_driver.
        """
        self.runningCV.acquire()

        self.running = False
        rospy.loginfo("wam_driver stopping!")

        self.runningCV.release()
        return []

    def computevm(self, ignore):
        res = self.dl()
        if res.loaded:
            model = self.dsparams()
            self.dT = model.model.dT
            rospy.loginfo("dT is %f" % self.dT)
            rospy.loginfo("Current rate is %d" % self.rateInt)

            cntl_dt = 1.0 / float(self.rateInt)
            newdt = cntl_dt / self.dT

            rospy.loginfo("Velocity multiplier should be %f" % newdt)
            rospy.loginfo("Current vm is %f" % self.vm)
        return []


    def start(self, ignore):
        """
        Call the start service to start the wam_driver.
        """

        self.runningCV.acquire()

        res = self.dl()
        if res.loaded:

            model = self.dsparams()
            rospy.loginfo("Dim %d" % model.model.dim)
            self.endpoint = npa(model.model.offset)[:model.model.dim/2]
            rospy.loginfo("Using endpoint %s" % str(self.endpoint))

            # init some variables
            self.x = list(self.current_pose.position)
            self.rot = list(self.current_pose.euler)
            self.newx = self.x

            self.running = True
            rospy.loginfo("wam_driver starting!")
            # need to send a signal to wake up the main thread?
        else:
            rospy.loginfo("ds_node model is not loaded -- not starting!")

        self.runningCV.notify()
        self.runningCV.release()
        return []

    def stepsrv(self, ignore):
        self.runningCV.acquire()
        self.step()
        self.runningCV.release()
        return []

    def step(self):
        self.compute_old_pose()
        self.compute_new_pose()

        rospy.logdebug("x : %s dx : %s newx : %s" % (str(self.x), str(self.dx), str(self.newx)))

        # set command
        self.cmd.position = self.newx
        self.cmd.euler = list(self.current_pose.euler)

        # publish
        self.pub.publish(self.cmd)

    def set_threshold(self, req):
        self.runningCV.acquire()
        self.adaptive_threshold = req.value
        self.runningCV.release()
        rospy.loginfo("Threshold set to %f" % self.adaptive_threshold)
        return []

    def compute_old_pose(self):

        """
        Seds sometimes produces velocities that are too small to
        overcome friction, leading to a frozen robot. Open loop
        control will overcome this friction (as the desired pose will
        begin to deviate from the actual pose and the command torques
        will rise). In the case of compliance, we have an adaptive
        feature that implements open loop locally, but updates using
        feedback if there is a big change to the robot pose (indicating
        perturbation by an outside source).
        """

        # TODO: record diff during no feedback mode

        nx = npa(self.newx)
        cp = npa(self.current_pose.position)
        rospy.loginfo("diff: %f" %  la.norm(nx - cp))

        if self.feedback == "adaptive":
            if la.norm(nx - cp) > self.adaptive_threshold:
                # something drastic has changed
                self.x = list(self.current_pose.position)
            else:
                self.x = self.newx

        elif self.feedback == "none":
            self.x = self.newx # old pose is last new pose
        elif self.feedback == "hard":
            self.x = list(self.current_pose.position) # old pose is robot's reported pose
        else:
            raise ValueError, "Unknown feedback! %s" % self.feedback

    def compute_new_pose(self):
        if self.useseds:
            self.dx = list(self.ds(self.x).dx)
            self.newx = list(npa(self.x) + self.vm * npa(self.dx))
        else: # just set the endpoint (might cause torque fault on some robots e.g. WAM)
            self.newx = self.endpoint

    def spin(self):

        rospy.loginfo("Running!")

        try:

            while not rospy.is_shutdown():

                self.runningCV.acquire()
                if self.running:

                    self.step()
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
    (options,args) = getopt.getopt(sys.argv[1:], 'v:f:', ['vm=','feedback='])

    rospy.init_node('wam_driver')

    vm = rospy.get_param("/wam_driver/velocity_multiplier", 1.0)
    feedback = rospy.get_param("/wam_driver/feedback", 'none')

    for o,a in options:
        if o in ('-v','--vm'):
            vm = float(a)
        elif o in ('-f','--feedback'):
            assert a in ('none','hard','adaptive')
            feedback = a

    driver = WAMDriver(vm, feedback, 500) # start node
    driver.spin()

if __name__ == '__main__':
    main()
