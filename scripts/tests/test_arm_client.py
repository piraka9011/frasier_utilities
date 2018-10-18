#!/usr/bin/env python

import rospy
from frasier_utilities.arm import Arm

try:
    rospy.init_node('arm_test', log_level=rospy.DEBUG)
    a = Arm()
    a.move_to_location('top_grasp')
except KeyboardInterrupt:
    rospy.signal_shutdown("Bye")
