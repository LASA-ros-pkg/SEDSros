#! /usr/bin/env python
"""
Author: Jeremy M. Stober
Program: REPLAY_TRAJ.PY
Date: Thursday, June  9 2011
Description: Replay a trajectory file.
"""

import time
import roslib; roslib.load_manifest('seds')
import rospy
from std_msgs.msg import String
from turtlesim.msg import Velocity,Pose
from turtlesim.srv import TeleportAbsolute

def teleport_robot(x,y,theta):
    rospy.wait_for_service('turtle1/teleport_absolute')
    try:
        teleport = rospy.ServiceProxy('turtle1/teleport_absolute', TeleportAbsolute)
        teleport(x,y,theta)
    except rospy.ServiceException, e:
        print "Service call failed! %s" % s

def parse(line):
    data = line.split(' ')
    return (float(data[0]), float(data[1]), float(data[2]),float(data[3]),float(data[4]))

def replay():

    rospy.init_node("replay", anonymous=True)
    pub = rospy.Publisher("turtle1/command_velocity", Velocity)

    trajfp = open("trajectories.txt")
    first = False
    for line in trajfp.readlines():

        if line[0] == 'T':
            first = True
            print line
            continue

        elif first is True:
            (x,y,theta,lv,av) = parse(line)
            teleport_robot(x,y,theta)
            first = False

        else:
            (x,y,theta,lv,av) = parse(line)
            pub.publish(lv,av)
            time.sleep(0.1)


if __name__ == '__main__':
    replay()


