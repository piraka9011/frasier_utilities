#!/usr/bin/env python

import rospy
from rospkg import RosPack
from actionlib import SimpleActionClient, GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, JointTolerance
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from geometry_msgs.msg import Point, Quaternion, PoseStamped, Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from hsr_constants import *

import time
from yaml import load, dump, YAMLError


class Base(object):
    def __init__(self):
        # Action Client
        self.move_client = SimpleActionClient(MOVE_BASE_TOPIC, MoveBaseAction)
        rospy.loginfo("BASE CLIENT: Waiting for base action server...")
        base_client_running = self.move_client.wait_for_server(rospy.Duration(3))
        if base_client_running:
            rospy.loginfo("BASE CLIENT: Base controller initialized.")
        else:
            rospy.loginfo("BASE CLIENT: Base controller is NOT initialized!")

        # Omni Client
        self.omni_client = SimpleActionClient(OMNI_BASE_CLIENT_TOPIC,
                                              FollowJointTrajectoryAction)
        omni_client_running = self.omni_client.wait_for_server(rospy.Duration(3))


        # Current Pose
        rospy.Subscriber(CURRENT_POSE_TOPIC, PoseStamped, self._pose_cb)
        self.current_pose = PoseStamped()

        # Velocity
        self.vel_pub = rospy.Publisher(BASE_VELOCITY_TOPIC, Twist, queue_size=10)

        # Default move_base goal init
        self.move_goal = MoveBaseGoal()
        self.move_goal.target_pose.header.frame_id = 'map'
        self.timeout = rospy.Duration(30)

        # Default traj goal init
        # Tolerances
        self.tolx = JointTolerance(name='odom_x', position=0.1)
        self.toly = JointTolerance(name='odom_y', position=0.1)
        self.tolt = JointTolerance(name='odom_t', position=0.1)
        self.traj_goal = FollowJointTrajectoryGoal()
        self.traj_goal.goal_tolerance = [self.tolx, self.toly, self.tolt]
        self.traj_goal.path_tolerance = [self.tolx, self.toly, self.tolt]
        self.traj_goal.trajectory.joint_names = OMNI_BASE_JOINTS
        self.traj_goal.trajectory.header.frame_id = 'map'

        r = RosPack()
        pkg_path = r.get_path('frasier_utilities')

        self.yaml_filename = pkg_path + '/config/wrs_map.yaml'
        with open(self.yaml_filename, 'r') as outfile:
            try:
                self.locations = load(outfile)
            except YAMLError as e:
                print e

    def _save_yaml_file(self):
        with open(self.yaml_filename, 'w') as outfile:
            dump(self.locations, outfile, default_flow_style=False)

    def read_yaml_file(self):
        with open(self.yaml_filename, 'r') as outfile:
            try:
                self.locations = load(outfile)
            except YAMLError as e:
                print e

    def _pose_cb(self, msg):
        self.current_pose = msg

    def _set_goal_position(self, p):
        self.move_goal.target_pose.pose.position = p

    def _set_goal_orientation(self, q):
        self.move_goal.target_pose.pose.orientation = q

    def _send_goal(self, goal=None, blocking=True):
        # Send built in goal, otherwise send passed goal
        if goal is None:
            self.move_goal.target_pose.header.stamp = rospy.Time.now()
            self.move_client.send_goal(self.move_goal)
        else:
            self.move_client.send_goal(goal)

        # Goal check procedure
        if blocking:
            rospy.loginfo("BASE CLIENT: Waiting for goal to complete...")
            result = self.move_client.wait_for_result(self.timeout)
            if not result:
                rospy.logwarn("BASE CLIENT: Goal timed out, canceled!")
                self.move_client.cancel_goal()
                return False
            else:
                state = self.move_client.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("BASE CLIENT: Goal completed.")
                    return True
                else:
                    rospy.logwarn("BASE CLIENT: Goal failed!")
                    return False
        # Assume it succeeded
        return True

    def move_to_relative(self, x, y, t, blocking=True):
        self.current_pose = rospy.wait_for_message(CURRENT_POSE_TOPIC, PoseStamped)
        rel_x = self.current_pose.pose.position.x + x
        rel_y = self.current_pose.pose.position.y + y
        q = self.current_pose.pose.orientation
        _, _, yaw = euler_from_quaternion((q.x, q.y, q.z, q.w))
        rel_t = yaw + t
        q = quaternion_from_euler(0, 0, rel_t)
        self.move_goal.target_pose.pose.position = Point(rel_x, rel_y, 0)
        self.move_goal.target_pose.pose.orientation = Quaternion(*q)
        self.move_goal.target_pose.header.frame_id = 'map'
        self._send_goal()

        # point = JointTrajectoryPoint()
        # point.positions = [x, y, t]
        # point.time_from_start = rospy.Duration(3)
        # self.traj_goal.trajectory.points = [point]
        # self.traj_goal.trajectory.header.stamp = rospy.Time.now()
        # print(self.traj_goal)
        # if not blocking:
        #     return self.omni_client.send_goal(self.traj_goal)
        # else:
        #     return self.omni_client.send_goal_and_wait(self.traj_goal)

    def move_to_location(self, requested_location, blocking=True):
        if type(requested_location) is not str:
            raise AttributeError('BASE CLIENT: move_to_location() expects a location string.')

        if requested_location in self.locations:
            rospy.loginfo("BASE CLIENT: Received {} location".format(requested_location))
            coords = self.locations[requested_location]
            self._set_goal_position(Point(coords['x'], coords['y'], 0))
            self._set_goal_orientation(Quaternion(0, 0, coords['z'], coords['w']))
            return self._send_goal(blocking=blocking)
        else:
            rospy.logwarn("BASE CLIENT: {} does not exist!".format(requested_location))
            return False

    def move_vel(self, vel=0.1, duration=0.0):
        vel_msg = Twist()
        vel_msg.linear.x = vel
        end = time.time() + duration
        while time.time() < end:
            self.vel_pub.publish(vel_msg)
