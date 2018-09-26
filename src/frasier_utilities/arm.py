#!/usr/bin/env python

import rospy
from rospkg import RosPack
from actionlib import SimpleActionClient, GoalStatus
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

from hsr_constants import ARM_CLIENT_TOPIC, ARM_JOINTS

import yaml


class Arm(object):
    def __init__(self):
        # Client
        self.arm_client = SimpleActionClient(ARM_CLIENT_TOPIC, FollowJointTrajectoryAction)
        rospy.loginfo("ARM CLIENT: Waiting for arm action server...")
        arm_client_running = self.arm_client.wait_for_server(rospy.Duration(2))
        if arm_client_running:
            rospy.loginfo("ARM CLIENT: Arm controller initialized.")
        else:
            rospy.loginfo("ARM CLIENT: Arm controller is NOT initialized!")

        # Setup
        self.arm_goal = FollowJointTrajectoryGoal()
        self.arm_goal.trajectory.joint_names = ARM_JOINTS
        self.p = JointTrajectoryPoint()
        self.timeout = rospy.Duration(30)

        # Pre-def. positions
        r = RosPack()
        dir = r.get_path('frasier_utilities') + '/config/arm_configs.yaml'
        with open(dir, 'r') as stream:
            try:
                self.positions = yaml.load(stream)
            except yaml.YAMLError as exc:
                rospy.logwarn("ARM CLIENT: Yaml Exception Caught: {}".format(exc))

        rospy.logdebug("ARM CLIENT: Config: {}".format(self.positions))

    def _send_goal(self, arm_goal=None, blocking=False):
        if arm_goal is None:
            self.arm_client.send_goal(self.arm_goal)
        else:
            self.arm_client.send_goal(arm_goal)

        if blocking:
            rospy.loginfo("ARM CLIENT: Waiting for goal to complete...")
            result = self.arm_client.wait_for_result(self.timeout)
            if not result:
                rospy.logwarn("ARM CLIENT: Goal timed out, canceled!")
                self.arm_client.cancel_goal()
                return False
            else:
                state = self.arm_client.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("ARM CLIENT: Goal completed.")
                    return True
                else:
                    rospy.logwarn("ARM CLIENT: Goal failed!")
                    return False
        # Assume it succeeded
        return True

    def send_positions(self, arm_position=None, blocking=False):
        if arm_position is None:
            self._send_goal(self.arm_goal, blocking)
        else:
            self.arm_goal.trajectory.points = [arm_position]
            self._send_goal(self.arm_goal, blocking)

    def goto_position(self, position):
        if position in self.positions:
            self.p.positions = self.positions[position]
            self.p.time_from_start = rospy.Time(3)
            self.arm_goal.trajectory.points = [self.p]
            self._send_goal()
        else:
            rospy.logwarn("ARM CLIENT: {} does not exist in the config file!".format(position))
