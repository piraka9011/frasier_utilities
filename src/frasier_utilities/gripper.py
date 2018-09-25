#!/usr/bin/env python
import rospy
from actionlib import SimpleActionClient, GoalStatus
from tmc_control_msgs.msg import GripperApplyEffortGoal, GripperApplyEffortAction
from hsr_constants import *


class Gripper(object):
    def __init__(self):
        self.gripper_client = SimpleActionClient(GRIPPER_CLIENT_TOPIC, GripperApplyEffortAction)
        self.gripper_client.wait_for_server(rospy.Duration(2))

    def grasp(self, effort=0.0):
        goal = GripperApplyEffortGoal()
        goal.effort = effort
        result = self.gripper_client.send_goal_and_wait(goal)
        if result ==GoalStatus.SUCCEEDED:
            return True
        else:
            return False

    def release(self):
        self.grasp(0.1)

    def grab(self):
        self.grasp(-0.3)
