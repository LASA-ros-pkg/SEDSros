#! /usr/bin/env python
"""
Author: Jeremy M. Stober
Program: GEN_TRAJ.PY
Date: Thursday, June  9 2011
Description: Generate turtle_sim trajectories for seds node.
"""

#!/usr/bin/env python
import roslib; roslib.load_manifest('seds')
import rospy
from std_msgs.msg import String
from turtlesim.msg import Velocity,Pose
from turtlesim.srv import TeleportAbsolute,SetPen

from seds.msg import SedsMessage

import numpy as np
import numpy.linalg as la
import numpy.random as npr
import math
import time
import rosbag
import rospy.rostime as rostime

startposes = [(2.0,2.0),(5.5,2.0),(7.5,2.0),(7.5,5.5),(7.5,7.5),(5.5,7.5),(2.0,7.5),(2.0,5.5)]

BASENAME = "/home/stober/workspace/ros/seds/turtle/bag"
GOAL = np.array([5.55,5.55,0.0])
CURRENT = np.zeros(3)
CURRENTPOSE = None
LASTPOSE = None
EPSILON = 0.1

def callback(data):
    global CURRENT
    global CURRENTPOSE
    global LASTPOSE
    CURRENT = np.array([data.x,data.y,data.theta])
    LASTPOSE = CURRENTPOSE
    CURRENTPOSE = (data,rostime.get_rostime())

def atgoal():
    return la.norm(GOAL[:2] - CURRENT[:2]) < EPSILON

def compute_velocity():

    if atgoal():
        return np.zeros(2)

    delta = (GOAL[:2] - CURRENT[:2])
    adelta = math.atan2(delta[1],delta[0])

    return 0.4, 0.0 #0.4 * (adelta - CURRENT[2])


def teleport_absolute_client(teleport, setpen,i):

    x,y = startposes[i]

    delta =  (GOAL[0] - x, GOAL[1] - y)
    adelta = math.atan2(delta[1],delta[0])
    theta = adelta

    try:
        teleport(x,y,theta)
        time.sleep(1)

    except rospy.ServiceException, e:
        print "Service call failed! %s" % s

def pose_to_seds(pose,lpose,i):
    x = [pose[0].x,pose[0].y]
    lx = None
    if lpose:
        lx = [lpose[0].x,lpose[0].y]
    else:
        lx = [0.0,0.0]
    dx = np.array(x) - np.array(lx)
    sm = SedsMessage()
    sm.x = x
    sm.dx = list(dx)
    sm.t = pose[1]
    if lpose:
        sm.dt = pose[1] - lpose[1]
    else:
        sm.dt = rostime.Duration(0)
    sm.index = i
    return sm

def controller():

    rospy.init_node('gen_traj', anonymous=True)

    # subscribe and publish to appropriate nodes
    rospy.Subscriber("turtle1/pose", Pose, callback)
    pub = rospy.Publisher("turtle1/command_velocity", Velocity)

    rospy.wait_for_service('turtle1/teleport_absolute')
    rospy.wait_for_service('turtle1/set_pen')

    teleport = rospy.ServiceProxy('turtle1/teleport_absolute', TeleportAbsolute)
    setpen = rospy.ServiceProxy('turtle1/set_pen', SetPen)

    time.sleep(1)

    sbag = rosbag.Bag(BASENAME + "_seds.bag", "w")

    # generate 8 trajectories
    for i in range(8):

        teleport_absolute_client(teleport, setpen,i)

        #bag = rosbag.Bag(BASENAME + "%d.bag" % i, "w")

        while not rospy.is_shutdown():

            v = compute_velocity()
            pub.publish(v[0],v[1])

            #bag.write('turtle1/pose', CURRENTPOSE[0])
            sbag.write('seds/trajectories', pose_to_seds(CURRENTPOSE,LASTPOSE,i))

            if atgoal():
                #bag.close()
                break

    sbag.close()

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException: pass
