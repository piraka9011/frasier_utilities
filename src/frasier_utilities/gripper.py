#!/usr/bin/env python

import rospy
from actionlib import SimpleActionClient, GoalStatus
from geometry_msgs.msg import WrenchStamped
from tmc_control_msgs.msg import GripperApplyEffortGoal, GripperApplyEffortAction
from hsr_constants import GRIPPER_CLIENT_TOPIC, GRIPPER_WRENCH_TOPIC

from time import time


class Gripper(object):
    def __init__(self):
        self.gripper_client = SimpleActionClient(GRIPPER_CLIENT_TOPIC, GripperApplyEffortAction)
        rospy.loginfo("GRIPPER CLIENT: Waiting for gripper action server...")
        self.gripper_client.wait_for_server(rospy.Duration(2))

        self.wrist_wrench = WrenchStamped()

        rospy.Subscriber(GRIPPER_WRENCH_TOPIC, WrenchStamped, self._wrench_cb)

    def _wrench_cb(self, msg):
        self.wrist_wrench = msg

    def grasp(self, effort=0.0):
        goal = GripperApplyEffortGoal()
        goal.effort = effort
        result = self.gripper_client.send_goal_and_wait(goal)
        if result == GoalStatus.SUCCEEDED:
            return True
        else:
            return False

    def release(self):
        rospy.loginfo("GRIPPER CLIENT: Releasing gripper.")
        self.grasp(0.1)

    def grab(self):
        rospy.loginfo("GRIPPER CLIENT: Closing gripper.")
        self.grasp(-0.3)

    def get_wrist_force(self):
        self.wrist_wrench = rospy.wait_for_message(GRIPPER_WRENCH_TOPIC, WrenchStamped)
        return self.wrist_wrench.wrench.force.x

    def wait_for_push(self, timeout=0.0):
        if timeout == 0.0:
            while not rospy.is_shutdown():
                x = self.wrist_wrench.wrench.force.x
                if x > 17:
                    return True
                rospy.Rate(10).sleep()
        else:
            end = time() + timeout
            while not rospy.is_shutdown() and (time() < end):
                x = self.wrist_wrench.wrench.force.x
                if x > 17:
                    return True
                rospy.Rate(10).sleep()

        return False
