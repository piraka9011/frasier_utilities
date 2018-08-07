#!/usr/bin/env python
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import rospy
from trajectory_msgs.msg import JointTrajectoryPoint
from math import pi
from hsr_constants import *


class ArmClient(object):
    def __init__(self):
        self.arm_client = actionlib.SimpleActionClient(ARM_CLIENT_TOPIC, FollowJointTrajectoryAction)

        self.arm_client.wait_for_server(rospy.Duration(2))
        self.arm_goal = FollowJointTrajectoryGoal()
        self.arm_goal.trajectory.joint_names = ARM_JOINTS
        self.p = JointTrajectoryPoint()

    def send_goal(self, arm_goal=None):
        if arm_goal is None:
            self.arm_client.send_goal(self.arm_goal)
        else:
            self.arm_client.send_goal(arm_goal)
        self.arm_client.wait_for_result()

    def send_positions(self, arm_position=None):
        if arm_position is None:
            self.arm_client.send_goal(self.arm_goal)
        else:
            self.arm_goal.trajectory.points = [arm_position]
            self.arm_client.send_goal(self.arm_goal)
        self.arm_client.wait_for_result()

    def spr_start(self):
        self.p.positions = [0.5, -2.5, 0, -1.0, 0.0]
        self.p.velocities = [0, 0, 0, 0, 0]
        self.p.time_from_start = rospy.Time(3)
        self.arm_goal.trajectory.points = [self.p]
        self.send_goal()

    def hmc_start(self):
        self.p.positions = [0.5, -2.5, 0, -1.0, 0]
        self.p.velocities = [0, 0, 0, 0, 0]
        self.p.time_from_start = rospy.Time(3)
        self.arm_goal.trajectory.points = [self.p]
        self.send_goal()

    def hmc_bag(self):
        self.p.positions = [0.0, 0.0, 0, -1, 1.5]
        self.p.velocities = [0, 0, 0, 0, 0]
        self.p.time_from_start = rospy.Time(3)
        self.arm_goal.trajectory.points = [self.p]
        self.send_goal()
