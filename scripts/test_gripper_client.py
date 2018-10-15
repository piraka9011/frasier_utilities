#!/usr/bin/env python
import rospy
from frasier_utilities import gripper

if __name__ == '__main__':
    rospy.init_node('test_gripper_client')
    g = gripper.Gripper()
    g.grab()