#!/usr/bin/env python

import rospy
from rospkg import RosPack
from actionlib import SimpleActionClient, GoalStatus
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectoryPoint

from hsr_constants import ARM_CLIENT_TOPIC, ARM_JOINTS, ARM_STATE_TOPIC

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

        # Subscribe for states
        rospy.Subscriber(ARM_STATE_TOPIC, JointTrajectoryControllerState, self._update_state)
        self.curr_arm_state_ = JointTrajectoryControllerState()

        # Setup
        self.arm_goal = FollowJointTrajectoryGoal()
        self.arm_goal.trajectory.joint_names = ARM_JOINTS
        self.p = JointTrajectoryPoint()
        self.timeout = rospy.Duration(30)

        # Pre-def. positions
        r = RosPack()
        self.yaml_filename = r.get_path('frasier_utilities') + '/config/arm_configs.yaml'
        with open(self.yaml_filename, 'r') as stream:
            try:
                self.positions = yaml.load(stream)
            except yaml.YAMLError as exc:
                rospy.logwarn("ARM CLIENT: Yaml Exception Caught: {}".format(exc))

        rospy.logdebug("ARM CLIENT: Config: {}".format(self.positions))

    def _update_state(self, msg):
        self.curr_arm_state_ = msg

    def _save_yaml_file(self):
        with open(self.yaml_filename, 'w') as outfile:
            yaml.dump(self.positions, outfile, default_flow_style=False)

    def _read_yaml_file(self):
        with open(self.yaml_filename, 'r') as outfile:
            try:
                self.positions = yaml.load(outfile)
            except yaml.YAMLError as e:
                print e

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

    def reset(self):
        self.goto_position('start', True)

    def save_position(self, position_name='default', arm_position=None, save_file=True):
        if arm_position is None:
            self.positions[position_name] = list(self.curr_arm_state_.actual.positions)
        else:
            if not isinstance(arm_position, list):
                raise TypeError("Arm positions must be of type list!")
            self.positions[position_name] = arm_position

        if save_file:
            self._save_yaml_file()

        return True

    def goto_position(self, position=None, blocking=False):
        self._read_yaml_file()
        # Goto yaml file position if string
        if isinstance(position, str):
            if position in self.positions:
                self.p.positions = self.positions[position]
                self.p.time_from_start = rospy.Time(3)
                self.arm_goal.trajectory.points = [self.p]
                return self._send_goal()
            else:
                rospy.logwarn("ARM CLIENT: {} does not exist in the config file!".format(position))
        elif isinstance(position, list):
            self.arm_goal.trajectory.points = position
            return self._send_goal(self.arm_goal, blocking)
        elif position is None :
            return self._send_goal(self.arm_goal, blocking)

    def __del__(self):
        rospy.loginfo("Shutting down arm client and saving locations")
        self._save_yaml_file()
