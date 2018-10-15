#!/usr/bin/env python

import rospy
from rospkg import RosPack
from actionlib import SimpleActionClient, GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, Quaternion, PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from hsr_constants import MOVE_BASE_TOPIC, CURRENT_POSE_TOPIC

from warnings import warn
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

        # Current Pose
        rospy.Subscriber(CURRENT_POSE_TOPIC, PoseStamped, self._pose_cb)
        self.current_pose = PoseStamped()

        # Default Goal init
        self.move_goal = MoveBaseGoal()
        self.move_goal.target_pose.header.frame_id = 'odom'
        self.timeout = rospy.Duration(30)

        r = RosPack()
        pkg_path = r.get_path('frasier_utilities')

        # ============================================================================================
        # Pre-def. positions
        warn("Old frasier locations object map and nav_client will be deprecated!")
        locations_path = pkg_path + '/config/locations.yaml'
        lo_map_path = pkg_path + '/config/location_object_map.yaml'
        with open(locations_path, 'r') as loc_stream, open(lo_map_path, 'r') as lo_stream:
            try:
                self.locations = load(loc_stream)
                self.location_object_map = load(lo_stream)
            except YAMLError as e:
                rospy.logwarn("BASE CLIENT: Yaml Exception Caught: {}".format(e))

        rospy.logdebug("BASE CLIENT: Config: {}\n{}".format(self.locations, self.location_object_map))
        # ============================================================================================

        self.yaml_filename = pkg_path + '/config/wrs_map.yaml'

    def _save_yaml_file(self):
        with open(self.yaml_filename, 'w') as outfile:
            dump(self.locations, outfile, default_flow_style=False)

    def read_yaml_file(self):
        with open(self.yaml_filename, 'r') as f:
            try:
                self.locations = load(f)
            except YAMLError as e:
                print e

    def _pose_cb(self, msg):
        self.current_pose = msg

    def _set_goal_position(self, p):
        self.move_goal.target_pose.pose.position = p

    def _set_goal_orientation(self, q):
        self.move_goal.target_pose.pose.orientation = q

    def _set_goal_from_location(self, pos):
        goal = MoveBaseGoal()
        goal.target_pose.pose.position.x = pos['x']
        goal.target_pose.pose.position.y = pos['y']
        goal.target_pose.pose.orientation.z = pos['z']
        goal.target_pose.pose.orientation.w = pos['w']
        goal.target_pose.header.frame_id = 'map'
        return goal

    def _send_goal(self, goal=None, blocking=True):
        # Send built in goal, otherwise send passed goal
        if goal is None:
            self.move_goal.target_pose.header.stamp = rospy.Time.now()
            print(self.move_goal)
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

    def move_to_coordinate(self, x, y, t):
        self._set_goal_position(Point(x, y, 0))
        quat = quaternion_from_euler(0, 0, t)
        self._set_goal_orientation(Quaternion(*quat))
        return self._send_goal()

    def move_to_location(self, requested_location):
        if type(requested_location) is not str:
            raise AttributeError('BASE CLIENT: move_to_location() expects a location string.')

        if requested_location in self.locations.keys():
            coords = self.locations[requested_location]
            self._set_goal_position(Point(coords['x'], coords['y'], 0))
            self._set_goal_orientation(Quaternion(0, 0, coords['z'], coords['w']))
            return self._send_goal()
        else:
            rospy.logwarn("BASE CLIENT: {} does not exist!".format(requested_location))
            return False

    def move_to(self, location):
        if location in self.locations:
            self.move_goal = self._set_goal_from_location(self.locations[location])
            return self._send_goal()
        else:
            rospy.logwarn("BASE CLIENT: {} location does not exist!".format(location))
            return False

    def move_to_relative(self, x, y, yaw):
        # Get the current pose euler to add
        q = self.current_pose.pose.orientation
        _, _, rel_yaw = euler_from_quaternion((q.x, q.y, q.z, q.w))
        relative_yaw = rel_yaw + yaw
        relative_quat = quaternion_from_euler(0, 0, relative_yaw)
        # Create the new relative goal
        self._set_goal_position(Point(self.current_pose.pose.position.x + x,
                                      self.current_pose.pose.position.y + y,
                                      0))
        self._set_goal_orientation(Quaternion(*relative_quat))
        self.move_goal.target_pose.header.frame_id = 'map'
        self._send_goal()

