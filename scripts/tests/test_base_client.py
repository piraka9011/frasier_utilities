#!/usr/bin/env python

import rospy
from frasier_utilities import base

if __name__ == '__main__':
    rospy.init_node('test_base_client')
    b = base.Base()
    # b.move_to_relative(-0.5, 0.0, 0.0)
    b.move_vel(vel=-0.1, duration=3)
