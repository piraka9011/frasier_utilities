#!/usr/bin/env python

# ROS
import rospy
from rospkg import RosPack
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectoryPoint
# HSR
from hsr_constants import *
# Other
import yaml


class HeadClient(object):
    def __init__(self):
        # Client
        self.head_client = actionlib.SimpleActionClient(HEAD_CLIENT_TOPIC, FollowJointTrajectoryAction)
        # State
        self.head_state = rospy.Subscriber(HEAD_STATE_TOPIC, JointTrajectoryControllerState, self.update_state)

        # Check
        head_client_running = self.head_client.wait_for_server(rospy.Duration(2))
        if head_client_running:
            print "HEAD CLIENT: Head controller initialized."
        else:
            print "HEAD CLIENT: Head controller is NOT initialized!"

        # Setup
        self.head_goal = FollowJointTrajectoryGoal()
        self.head_goal.trajectory.joint_names = HEAD_JOINTS
        self.p = JointTrajectoryPoint()
        self.current_tilt = 0.0
        self.current_pan = 0.0

        # Pre-def. positions
        r = RosPack()
        fpath = r.get_path('frasier_utilities') + '/config/head_configs.yaml'
        with open(fpath, 'r') as stream:
            try:
                self.positions = yaml.load(stream)
            except yaml.YAMLError as exc:
                rospy.logwarn("Yaml Exception Caught: {}".format(exc))

        rospy.logdebug("Config: {}".format(self.positions))

    def update_state(self, msg):
        self.current_tilt, self.current_pan = msg.actual.positions

    def send_goal(self, head_goal=None, blocking=True):
        if head_goal is None:
            self.head_client.send_goal(self.head_goal)
        else:
            self.head_client.send_goal(head_goal)

        if blocking:
            self.head_client.wait_for_result()

    def set_point(self, point=None):
        if point is None:
            self.head_goal.trajectory.points = [self.p]
        else:
            self.head_goal.trajectory.points = [point]

    def goto_position(self, position, time=3, blocking=True):
        if position in self.positions:
            self.p.positions = self.positions[position]
            self.p.time_from_start = rospy.Time(time)
            self.set_point()
            self.send_goal(blocking=blocking)
        else:
            rospy.logwarn("This position does not exist in the config file! Please make one.")

    # Pan: [-3.84,1.75] [right,left]
    # Tilt: [-1.57,0.52] [down,up]
    def move(self, pan=None, tilt=None, velocities=None, move_time=1, blocking=True):
        if pan is None:
            pan = self.current_pan
        if tilt is None:
            tilt = self.current_tilt
        if velocities is None:
            velocities = [0, 0]

        self.p.positions = [pan, tilt]
        self.p.velocities = velocities
        self.p.time_from_start = rospy.Time(move_time)
        self.set_point()
        self.send_goal(blocking=blocking)

    def reset(self):
        self.p.positions = [0, 0]
        self.p.velocities = [0, 0]
        self.p.time_from_start = rospy.Time(2)
        self.set_point()
        self.send_goal()
