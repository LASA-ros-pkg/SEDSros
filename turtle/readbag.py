#! /usr/bin/env python
"""
Author: Jeremy M. Stober
Program: READBAG.PY
Date: Monday, June 13 2011
Description: simple way to read bag data
"""

import roslib; roslib.load_manifest('seds')
import rospy
import rosbag
bag = rosbag.Bag('bag0.bag')
for topic, msg, t in bag.read_messages():
    print t
bag.close()

