#!/usr/bin/env python

import rospy
from frasier_utilities.head import Head
from math import pi

try:
    rospy.init_node('head_test')
    h = Head()
    # h.reset()
    # h.move_to_location('tri_demo', 2)
    h.move_to_position(-pi/4, pi/12)
except KeyboardInterrupt:
    rospy.signal_shutdown("Bye")
