#! /usr/bin/env python
"""
Author: Jeremy M. Stober
Program: ROBOT.PY
Date: Thursday, July 14 2011
Description: Pipe one Point topic into another.
"""
import roslib
roslib.load_manifest('seds')
import rospy
import sys
import json

from geometry_msgs.msg import Point

data = None
def callback(ndata):
    global data
    data = ndata

if __name__ == '__main__':

    global data

    initial_value = eval(sys.argv[1])
    itopic = sys.argv[2]
    otopic = sys.argv[3]

    rospy.init_node('robot')

    data = Point()
    data.x = initial_value[0]
    data.y = initial_value[1]
    data.z = initial_value[2]

    sub = rospy.Subscriber(itopic,Point, callback)
    pub = rospy.Publisher(otopic,Point)

    while not rospy.is_shutdown():
        # just publish what you receive
        pub.publish(data)

