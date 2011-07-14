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
import driver

import rospy
import rospy.rostime as rostime

from wam_msgs.msg import CartesianCoordinates

import numpy
import numpy.linalg as la
import getopt
import sys
npa = numpy.array


class WAMDriver(driver.Driver):

    def callback(self, data):
        self.runningCV.acquire()
        self.current_pose = data
        self.runningCV.release()

    def init_publisher(self):
        self.cmd = CartesianCoordinates()
        self.pub = rospy.Publisher('/wam/cartesian_command', CartesianCoordinates)

    def init_subscriber(self):
        self.current_pose = CartesianCoordinates()
        self.sub = rospy.Subscriber('/wam/cartesian_coordinates', CartesianCoordinates, self.callback)

    def __init__(self, name, vm, feedback, rate):
        driver.Driver.__init__(self, name, vm, feedback, rate)
        # custom stuff here if needed

    def init_start(self):

        self.model = self.dsparams()
        rospy.loginfo("Dim %d" % self.model.model.dim)
        self.endpoint = npa(self.model.model.offset)[:self.model.model.dim/2]
        rospy.loginfo("Using endpoint %s" % str(self.endpoint))

        cntl_dt = 1.0 / float(self.rateInt)
        self.tscale = cntl_dt / self.dT

        rospy.loginfo("model dt: %s cntl dt: %s tscale : %s" % (self.dT, cntl_dt, self.tscale))
        rospy.loginfo("Dim %d" % self.model.model.dim)
        rospy.loginfo("Using endpoint %s" % str(self.endpoint))

        # init some variables
        self.x = list(self.current_pose.position)
        self.rot = list(self.current_pose.euler)
        self.newx = self.x

    def get_current_position(self):
        return list(self.current_pose.position)

    def publish(self):
        rospy.logdebug("x : %s dx : %s newx : %s" % (str(self.x), str(self.dx), str(self.newx)))

        # set command
        self.cmd.position = self.newx
        self.cmd.euler = list(self.current_pose.euler)

        # publish
        self.pub.publish(self.cmd)

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

    driver = WAMDriver('wam_driver', vm, feedback, 500) # start node
    driver.spin()

if __name__ == '__main__':
    main()
