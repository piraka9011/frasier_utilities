#!/usr/bin/env python
import rospy
from actionlib import SimpleActionClient, GoalStatus
from tmc_control_msgs.msg import GripperApplyEffortGoal, GripperApplyEffortAction
from hsr_constants import GRIPPER_CLIENT_TOPIC


class Gripper(object):
    def __init__(self):
        self.gripper_client = SimpleActionClient(GRIPPER_CLIENT_TOPIC, GripperApplyEffortAction)
        rospy.loginfo("GRIPPER CLIENT: Waiting for gripper action server...")
        self.gripper_client.wait_for_server(rospy.Duration(2))

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
