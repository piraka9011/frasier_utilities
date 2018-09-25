#!/usr/bin/env python

import rospy
from frasier_utilities.head import Head

try:
    rospy.init_node('head_test')
    h = Head()
    h.reset()
    h.goto_position('tri_demo', 2)
except KeyboardInterrupt:
    rospy.signal_shutdown("Bye")
