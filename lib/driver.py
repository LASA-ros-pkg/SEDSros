#! /usr/bin/env python
"""
Author: Jeremy M. Stober
Program: DRIVER.PY
Date: Thursday, July 14 2011
Description: A generic driver class so that PR2 and WAM drivers (and other drivers) can share code.

To subclass this class:

Implement all init_* methods to create the necessary publishers (for
commands) and subscribers (for position information) and the code that
initializes all the variables at the beginning of every run loop
(init_start).

Implement get_current_position which takes the subscribed information
(in some type) and turns it into a list of floats for sending to
ds_node (the model).

Implement publish which takes the current computed new position (newx)
and formats it for publishing on the command topic.

That's it!

Some notes: Both pr2 and wam drivers shared the same basic structure
which is captured in this parent class. The main loop logic is

1. Read from some position topic.

2. Do some logic to update the current position (self.x) using either
the read position or the last position command.

3. Send this computed position to seds.

4. Compute a new position command (self.newx) and format it for
publishing to the command topic.


"""

import roslib
roslib.load_manifest('seds')

import rospy
import rospy.rostime as rostime
from seds.srv import DSSrv
from seds.srv import DSLoaded
from seds.srv import FloatSrv, IntSrv, StringSrv
from seds.srv import SedsModel
from geometry_msgs.msg import Point
from std_srvs.srv import Empty


import numpy
import numpy.linalg as la
import getopt
import sys
npa = numpy.array

import threading

class Driver(object):

    def __init__(self, name, vm, feedback, rate):

        rospy.loginfo("Initialized with vm: %f and feedback: %s" % (vm, str(feedback)))

        self.useseds = True
        self.vm = vm
        self.tscale = 1.0
        self.dT = 0.01
        self.feedback = feedback
        self.rateInt = rate
        self.rate = rospy.Rate(rate)
        self.name = name

        # wait for the ds server
        rospy.loginfo("Waiting for ds_node...")
        rospy.wait_for_service("/ds_node/ds_server")
        rospy.loginfo("ds_node found!")

        self.dl = rospy.ServiceProxy("/ds_node/is_loaded", DSLoaded) # check whether the ds_server has a model
        self.ds = rospy.ServiceProxy("/ds_node/ds_server", DSSrv) # the running model
        self.dsparams = rospy.ServiceProxy("/ds_node/params", SedsModel)

        self.startSRV = rospy.Service("/%s/start" % name, Empty, self.start)
        self.stopSRV = rospy.Service("/%s/stop" % name, Empty, self.stop)
        self.quitSRV = rospy.Service("/%s/quit" % name, Empty, self.quit)
        self.vmSRV = rospy.Service("/%s/set_vm" % name, FloatSrv, self.set_vm)
        self.rateSRV = rospy.Service("/%s/set_rate" % name, IntSrv, self.set_rate)
        self.fbSRV = rospy.Service("/%s/change_feedback" % name, StringSrv, self.change_feedback)
        self.toggleSRV = rospy.Service("/%s/toggle_seds" % name, Empty, self.toggle_seds)
        self.stepSRV = rospy.Service("/%s/step" % name, Empty, self.stepsrv)
        self.thresholdSRV = rospy.Service("/%s/set_threshold" % name, FloatSrv, self.set_threshold)
        self.statusSRV = rospy.Service("/%s/status" % name, Empty, self.show_status) # prints out status to ros_info

        self.zerot = rostime.Time(0)

        self.adaptive_threshold = 0.1 # 10 cm

        self.running = False
        self.runningCV = threading.Condition()

        self.init_publisher() # creates self.pub
        self.init_subscriber() # creates self.sub

    def init_publisher(self):
        """
        Override this in subclass.
        """
        self.pub = rospy.Publisher("/robot/publish", Point)

    def callback(self, data):
        """
        Reads subscriber data.
        """
        self.runningCV.acquire()
        self.sdata = data
        self.runningCV.release()

    def init_subscriber(self):
        """
        Override this in subclass.
        """
        self.sub = rospy.Subscriber("/robot/subscribe", Point, self.callback)


    def toggle_seds(self, ignore):
        self.runningCV.acquire()
        self.useseds = not self.useseds
        self.runningCV.release()
        rospy.loginfo("Toggling seds to %s" % self.useseds)
        return []

    def set_vm(self, req):
        self.runningCV.acquire()
        self.vm = req.value
        self.runningCV.release()
        rospy.loginfo("VM set to %f" % self.vm)
        return []

    def set_rate(self, req):
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
        Call the quit service to quit the driver.
        """
        self.runningCV.acquire()
        self.running = False
        rospy.core.signal_shutdown("quit %s" % self.name)
        self.runningCV.release()
        return []


    def stop(self, ignore):
        """
        Call the stop service to stop the driver.
        """
        self.runningCV.acquire()

        self.running = False
        rospy.loginfo("%s stopping!" % self.name)

        self.runningCV.release()
        return []

    def show_status(self, ignore):
        rospy.loginfo("Name: %s" % self.name)
        rospy.loginfo("useseds: %s" % str(self.useseds))
        rospy.loginfo("vm: %s" % str(self.vm))
        rospy.loginfo("feedback: %s" % str(self.feedback))
        rospy.loginfo("rate: %s" % str(self.rateInt))
        rospy.loginfo("dT: %s" % str(self.dT))
        rospy.loginfo("tscale: %s" % str(self.tscale))
        rospy.loginfo("thresh: %s" % str(self.adaptive_threshold))
        return []

    def stepsrv(self, ignore):
        self.runningCV.acquire()
        self.step()
        self.runningCV.release()
        return []

    def init_start(self):
        """
        Called inside start to make sure everything is properly initialized.
        Override!
        """

        self.model = self.dsparams()
        self.endpoint = npa(self.model.model.offset)[:self.model.model.dim/2]
        self.dT = self.model.model.dT

        cntl_dt = 1.0 / float(self.rateInt)
        self.tscale = cntl_dt / self.dT

        rospy.loginfo("model dt: %s cntl dt: %s tscale : %s" % (self.dT, cntl_dt, self.tscale))
        rospy.loginfo("Dim %d" % self.model.model.dim)
        rospy.loginfo("Using endpoint %s" % str(self.endpoint))

        self.x = [self.sdata.x, self.sdata.y, self.sdata.z]
        self.newx = self.x

    def start(self, ignore):
        """
        Sets running to true and inits a number of important variables.
        """

        self.runningCV.acquire()
        res = self.dl()
        if res.loaded:
            # init some variables
            self.init_start()

            self.running = True
            rospy.loginfo("%s starting!" % self.name)
        else:
            rospy.loginfo("ds_node model not loaded -- not starting!")

        self.runningCV.notify()
        self.runningCV.release()
        return []

    def publish(self):
        pt = Point()
        pt.x = self.newx[0]
        pt.y = self.newx[1]
        pt.z = self.newx[2]
        self.pub.publish(pt)
        rospy.logdebug("x : %s dx : %s newx : %s" % (str(self.x), str(self.dx), str(self.newx)))

    def step(self):
        self.compute_old_pose() # incorporate feedback
        self.compute_new_pose() # using seds
        #print self.x + self.newx
        self.publish() # publish the command

    def set_threshold(self, req):
        self.runningCV.acquire()
        self.adaptive_threshold = req.value
        self.runningCV.release()
        rospy.loginfo("Threshold set to %f" % self.adaptive_threshold)
        return []

    def get_current_position(self):
        return [self.sdata.x, self.sdata.y, self.sdata.z]

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

        position = self.get_current_position()
        nx = npa(self.newx)
        cp = npa(position)
        rospy.logdebug("diff: %f" %  la.norm(nx - cp))

        if self.feedback == "adaptive":
            if la.norm(nx - cp) > self.adaptive_threshold:
                # something drastic has changed
                self.x = position
            else:
                self.x = self.newx

        elif self.feedback == "none":
            self.x = self.newx # old pose is last new pose
        elif self.feedback == "hard":
            self.x = list(position) # old pose is robot's reported pose
        else:
            raise ValueError, "Unknown feedback! %s" % self.feedback

    def compute_new_pose(self):
        if self.useseds:
            self.dx = list(self.ds(self.x).dx)
            # tscale accounts for timing differences, vm is a hack to overcome friction
            self.newx = list(npa(self.x) + self.vm * self.tscale * npa(self.dx))
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

    rospy.init_node('test_driver')

    vm = rospy.get_param("/test_driver/velocity_multiplier", 1.0)
    feedback = rospy.get_param("/test_driver/feedback", 'hard')

    for o,a in options:
        if o in ('-v','--vm'):
            vm = float(a)
        elif o in ('-f','--feedback'):
            assert a in ('none','hard','adaptive')
            feedback = a

    driver = Driver("test_driver", vm, feedback, 100) # start node
    driver.spin()

if __name__ == '__main__':
    main()
