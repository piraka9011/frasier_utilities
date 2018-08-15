#!/usr/bin/env python
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import rospy
from trajectory_msgs.msg import JointTrajectoryPoint
from hsr_constants import *
import yaml
from rospkg import RosPack


class ArmClient(object):
    def __init__(self):
        # Client
        self.arm_client = actionlib.SimpleActionClient(ARM_CLIENT_TOPIC, FollowJointTrajectoryAction)

        # Check
        arm_client_running = self.arm_client.wait_for_server(rospy.Duration(2))
        if arm_client_running:
            print "ARM CLIENT: Arm controller initialized."
        else:
            print "ARM CLIENT: Arm controller is NOT initialized!"

        # Setup
        self.arm_goal = FollowJointTrajectoryGoal()
        self.arm_goal.trajectory.joint_names = ARM_JOINTS
        self.p = JointTrajectoryPoint()

        # Pre-def. positions
        r = RosPack()
        dir = r.get_path('frasier_utilities') + '/config/arm_configs.yaml'
        with open(dir, 'r') as stream:
            try:
                self.positions = yaml.load(stream)
            except yaml.YAMLError as exc:
                rospy.logwarn("Yaml Exception Caught: {}".format(exc))

        rospy.logdebug("Config: {}".format(self.positions))

    def send_goal(self, arm_goal=None, wait=False):
        if arm_goal is None:
            self.arm_client.send_goal(self.arm_goal)
        else:
            self.arm_client.send_goal(arm_goal)

        if wait:
            self.arm_client.wait_for_result()

    def send_positions(self, arm_position=None, wait=False):
        if arm_position is None:
            self.arm_client.send_goal(self.arm_goal)
        else:
            self.arm_goal.trajectory.points = [arm_position]
            self.arm_client.send_goal(self.arm_goal)

        if wait:
            self.arm_client.wait_for_result()

    def goto_position(self, position):
        if position in self.positions:
            self.p.positions = self.positions[position]
            self.p.time_from_start = rospy.Time(3)
            self.arm_goal.trajectory.points = [self.p]
            self.send_goal()
        else:
            rospy.logwarn("This position does not exist in the config file! Please make one.")
