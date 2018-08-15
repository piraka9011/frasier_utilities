#!/usr/bin/env python

import rospy
from frasier_utilities.head_client import HeadClient

try:
    rospy.init_node('head_test')
    h = HeadClient()
    h.reset()
    h.goto_position('tri_demo', 2)
except KeyboardInterrupt:
    rospy.signal_shutdown("Bye")
