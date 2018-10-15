#!/usr/bin/env python

import rospy
from frasier_utilities import base

if __name__ == '__main__':
    rospy.init_node('test_base_client')
    b = base.Base()
    b.move_to_relative(-0.25, 0.0, 0.0)
