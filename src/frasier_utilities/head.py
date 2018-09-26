#!/usr/bin/env python

# ROS
import rospy
from rospkg import RosPack
from actionlib import GoalStatus, SimpleActionClient
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectoryPoint
# HSR
from hsr_constants import HEAD_CLIENT_TOPIC, HEAD_STATE_TOPIC, HEAD_JOINTS
# Other
import yaml


class Head(object):
    def __init__(self):
        self.head_client = SimpleActionClient(HEAD_CLIENT_TOPIC, FollowJointTrajectoryAction)
        self.head_state = rospy.Subscriber(HEAD_STATE_TOPIC, JointTrajectoryControllerState, self.update_state)
        rospy.loginfo("HEAD CLIENT: Waiting for head action server...")
        head_client_running = self.head_client.wait_for_server(rospy.Duration(2))
        if head_client_running:
            rospy.loginfo("HEAD CLIENT: Head controller initialized.")
        else:
            rospy.loginfo("HEAD CLIENT: Head controller is NOT initialized!")

        # Setup
        self.head_goal = FollowJointTrajectoryGoal()
        self.head_goal.trajectory.joint_names = HEAD_JOINTS
        self.p = JointTrajectoryPoint()
        self.current_tilt = 0.0
        self.current_pan = 0.0
        self.timeout = rospy.Duration(30)

        # Pre-def. positions
        r = RosPack()
        fpath = r.get_path('frasier_utilities') + '/config/head_configs.yaml'
        with open(fpath, 'r') as stream:
            try:
                self.positions = yaml.load(stream)
            except yaml.YAMLError as exc:
                rospy.logwarn("HEAD CLIENT: Yaml Exception Caught: {}".format(exc))

        rospy.logdebug("HEAD CLIENT: Config: {}".format(self.positions))

    def update_state(self, msg):
        self.current_tilt, self.current_pan = msg.actual.positions

    def send_goal(self, head_goal=None, blocking=True):
        if head_goal is None:
            self.head_goal.trajectory.header.stamp = rospy.Time.now()
            self.head_client.send_goal(self.head_goal)
        else:
            self.head_client.send_goal(head_goal)

        if blocking:
            rospy.loginfo("HEAD CLIENT: Waiting for goal to complete...")
            result = self.head_client.wait_for_result(self.timeout)
            if not result:
                rospy.logwarn("HEAD CLIENT: Goal timed out, canceled!")
                self.head_client.cancel_goal()
                return False
            else:
                state = self.head_client.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("HEAD CLIENT: Goal completed.")
                    return True
                else:
                    rospy.logwarn("HEAD CLIENT: Goal failed!")
                    return False
            # Assume it succeeded
        return True

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
            rospy.logwarn("HEAD CLIENT: {} does not exist in the config file!".format(position))

    # Pan: [-3.84,1.75] [right,left]
    # Tilt: [-1.57,0.52] [down,up]
    def move(self, pan=None, tilt=None, velocities=None, move_time=1, blocking=True):
        if pan is None:
            pan = self.current_pan
        if tilt is None:
            tilt = self.current_tilt
        if velocities is None:
            velocities = [0, 0]

        rospy.loginfo("HEAD CLIENT: Moving to {}, {} (Pan, Tilt)".format(pan, tilt))
        self.p.positions = [pan, tilt]
        self.p.velocities = velocities
        self.p.time_from_start = rospy.Time(move_time)
        self.set_point()
        self.send_goal(blocking=blocking)

    def reset(self):
        rospy.loginfo("HEAD CLIENT: Resetting head.")
        self.p.positions = [0, 0]
        self.p.velocities = [0, 0]
        self.p.time_from_start = rospy.Time(2)
        self.set_point()
        self.send_goal()
