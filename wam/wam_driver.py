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
from seds.srv import FloatSrv, IntSrv
from seds.srv import SedsModel
from std_srvs.srv import Empty

import numpy
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
        self.rate = rate

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
        self.rateSRV = rospy.Service('/wam_driver/setrate', IntSrv, self.setrate)
        self.fbSRV = rospy.Service('/wam_driver/toggle_feedback', Empty, self.toggleFb)
        self.toggleSRV = rospy.Service('/wam_driver/toggle_seds', Empty, self.toggleSeds)

        self.zerot = rostime.Time(0)


        self.cmd = CartesianCoordinates()
        self.current_pose = CartesianCoordinates()

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

    def spin(self):

        rospy.loginfo("Running!")

        try:

            while not rospy.is_shutdown():

                self.runningCV.acquire()
                if self.running:

                    # if feedback is true then re-intialize x on every loop using tf
                    self.rot = list(self.current_pose.euler)

                    if self.feedback:
                        self.x = list(self.current_pose.position)
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

                    self.cmd.position = self.newx
                    self.cmd.euler = self.rot

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
    (options,args) = getopt.getopt(sys.argv[1:], 'v:f', ['vm=','feedback'])

    rospy.init_node('wam_driver')

    vm = rospy.get_param("/wam_driver/velocity_multiplier", 1.0)
    feedback = rospy.get_param("/wam_driver/feedback", True)

    for o,a in options:
        if o in ('-v','--vm'):
            vm = float(a)
        elif o in ('-f','--feedback'):
            feedback = True

    driver = WAMDriver(vm, feedback, rospy.Rate(100)) # start node
    driver.spin()

if __name__ == '__main__':
    main()
