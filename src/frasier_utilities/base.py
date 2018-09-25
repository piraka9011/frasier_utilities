#!/usr/bin/env python
import rospy
from rospkg import RosPack
from actionlib import SimpleActionClient, GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Point, Quaternion

from hsr_constants import *

from yaml import load, YAMLError


class Base(object):
    def __init__(self):
        self.move_client = SimpleActionClient(MOVE_BASE_TOPIC, MoveBaseAction)
        self.move_client.wait_for_server(rospy.Duration(3))
        self.move_goal = MoveBaseGoal()
        self.move_goal.target_pose.header.frame_id = 'odom'

        # Pre-def. positions
        r = RosPack()
        pkg_path = r.get_path('frasier_utilities')
        locations_path = pkg_path + '/config/locations.yaml'
        lo_map_path = pkg_path + '/config/location_object_map.yaml'
        with open(locations_path, 'r') as loc_stream, open(lo_map_path, 'r') as lo_stream:
            try:
                self.locations = load(loc_stream)
                self.location_object_map = load(lo_stream)
            except YAMLError as e:
                rospy.logwarn("Yaml Exception Caught: {}".format(e))

        rospy.logdebug("Config: {}\n{}".format(self.locations, self.location_object_map))

        self.timeout = rospy.Duration(30)

    def set_goal_position(self, p):
        self.move_goal.target_pose.pose.position = p

    def set_goal_orientation(self, q):
        self.move_goal.target_pose.pose.orientation = q

    def send_goal(self, goal=None, blocking=True):
        # Send built in goal, otherwise send passed goal
        if goal is None:
            self.move_goal.target_pose.header.stamp = rospy.Time.now()
            self.move_client.send_goal(self.move_goal)
        else:
            self.move_client.send_goal(goal)

        # Goal check procedure
        if blocking:
            rospy.loginfo("Base: Waiting for goal to complete...")
            result = self.move_client.wait_for_result(self.timeout)
            if not result:
                rospy.logwarn("Base: Goal timed out, canceled!")
                self.move_client.cancel_goal()
                return False
            else:
                state = self.move_client.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Base: Goal completed.")
                    return True
                else:
                    rospy.logwarn("Base: Goal failed!")
                    return False
        # Assume it succeeded
        return True

    def move_to_coordinate(self, x, y, t):
        self.set_goal_position(Point(x, y, 0))
        quat = quaternion_from_euler(0, 0, t)
        self.set_goal_orientation(Quaternion(*quat))
        return self.send_goal()

    def move_to_location(self, requested_location):
        if requested_location in self.locations.keys():
            coords = self.locations[requested_location]
            self.set_goal_position(Point(coords['x'], coords['y'], 0))
            self.set_goal_orientation(Quaternion(0, 0, coords['z'], coords['w']))
            return self.send_goal()
        else:
            rospy.logwarn("Base: {} does not exist!".format(requested_location))
            return False



