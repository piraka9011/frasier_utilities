#!/usr/bin/env python

import rospy
from frasier_utilities.arm_client import ArmClient

try:
    rospy.init_node('arm_test', log_level=rospy.DEBUG)
    a = ArmClient()
    a.goto_position('top_grasp')
except KeyboardInterrupt:
    rospy.signal_shutdown("Bye")
