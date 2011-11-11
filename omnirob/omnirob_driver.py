#! /usr/bin/env python
"""
Author: Christophe Paccolat
Program: OMNIROB_DRIVER.PY
Date: Friday, August 26 2011
Description: Publishes ds commands to /omnirob/jointangles_command.
"""

import roslib
roslib.load_manifest('seds')
roslib.load_manifest('orca_proxy')
import driver

import rospy
import rospy.rostime as rostime

from orca_proxy.msg import joints_set
from orca_proxy.msg import joints_fb

import numpy
import numpy.linalg as la
import getopt
import sys
npa = numpy.array


class OmnirobDriver(driver.Driver):

    def callback(self, data):
        self.runningCV.acquire()
        self.current_pose = data
        self.runningCV.release()

    # Publish to
    def init_publisher(self):
        self.cmd = joints_set()
        self.pub = rospy.Publisher('/joints_set', joints_set)
    
    # Subscribe to
    def init_subscriber(self):
        self.current_pose = joints_fb()
        self.sub = rospy.Subscriber('/joints_fb', joints_fb, self.callback)

    def __init__(self, name, vm, feedback, rate):
        driver.Driver.__init__(self, name, vm, feedback, rate)
        # custom stuff here if needed

    def init_start(self):

        self.model = self.dsparams()
        self.endpoint = npa(self.model.model.offset)[:self.model.model.dim/2]
        self.dT = self.model.model.dT

        cntl_dt = 1.0 / float(self.rateInt)
        self.tscale = cntl_dt / self.dT

        rospy.loginfo("model dt: %s cntl dt: %s tscale : %s" % (self.dT, cntl_dt, self.tscale))
        rospy.loginfo("Dim %d" % self.model.model.dim)
        rospy.loginfo("Using endpoint %s" % str(self.endpoint))

        # init some variables
        self.x = list(self.current_pose.position)
        self.newx = self.x
        return True

    def get_current_position(self):
        return list(self.current_pose.position)

    def publish(self):
        rospy.logdebug("x : %s dx : %s newx : %s" % (str(self.x), str(self.dx), str(self.newx)))

        # set command
        self.cmd.action = [7]*9 # goto abs position
        self.cmd.maxforce = [10]*9
        self.cmd.controlid = [8, 8, 7, 6, 5, 4, 3, 2, 1]
        self.cmd.targetpos = self.newx 
        # publish
        self.pub.publish(self.cmd)

def main():

    # rospy gets first crack at sys.argv
    rospy.myargv(argv=sys.argv)
    (options,args) = getopt.getopt(sys.argv[1:], 'v:f:', ['vm=','feedback='])

    rospy.init_node('omnirob_driver')

    vm = rospy.get_param("/omnirob_driver/velocity_multiplier", 1.0)
    feedback = rospy.get_param("/omnirob_driver/feedback", 'none')

    for o,a in options:
        if o in ('-v','--vm'):
            vm = float(a)
        elif o in ('-f','--feedback'):
            assert a in ('none','hard','adaptive')
            feedback = a

    driver = OmnirobDriver('omnirob_driver', vm, feedback, 500) # start node
    driver.spin()

if __name__ == '__main__':
    main()
