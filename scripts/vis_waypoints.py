#!/usr/bin/env python

import rospy
from rospkg import RosPack
from visualization_msgs.msg import MarkerArray, Marker

import signal
from yaml import load, YAMLError


def signal_handler(sig, frame):
    rospy.signal_shutdown("Ctrl-C")
    rospy.loginfo("Ctrl-C")
    exit(0)


class WaypointVisualizer(object):

    def __init__(self):
        self.waypoint_pub = rospy.Publisher('/frasier_visualization/map_waypoints', MarkerArray, queue_size=100)
        self.text_pub = rospy.Publisher('/frasier_visualization/map_text', MarkerArray, queue_size=100)
        self.arrow_marker_array = MarkerArray()
        self.text_marker_array = MarkerArray()

        r = RosPack()
        yaml_filename = r.get_path('frasier_utilities') + '/config/wrs_map.yaml'
        with open(yaml_filename, 'r') as outfile:
            try:
                self.map_locations = load(outfile)
            except YAMLError:
                rospy.logwarn("Could not load the yaml file waypoints!")
        rospy.loginfo("Waypoint Visualizer: Ready!")

    def create_marker_array(self):
        rospy.loginfo("Waypoint Visualizer: Creating marker array...")
        arrow_list = []
        text_list = []
        id = 0
        for location, pose in self.map_locations.iteritems():
            # Arrows
            arrow_marker = Marker()
            id += 1
            arrow_marker.header.frame_id = 'map'
            arrow_marker.ns = '/frasier_visualization'
            arrow_marker.id = id
            arrow_marker.type = arrow_marker.ARROW
            arrow_marker.scale.x = 1
            arrow_marker.scale.y = 0.1
            arrow_marker.scale.z = 0.1
            arrow_marker.color.a = 1.0
            arrow_marker.color.r = 1.0
            arrow_marker.color.g = 0.0
            arrow_marker.color.b = 0.0
            arrow_marker.pose.position.x = pose['x']
            arrow_marker.pose.position.y = pose['y']
            arrow_marker.pose.position.z = 0.0
            arrow_marker.pose.orientation.x = 0.0
            arrow_marker.pose.orientation.y = 0.0
            arrow_marker.pose.orientation.z = pose['z']
            arrow_marker.pose.orientation.w = pose['w']
            arrow_list.append(arrow_marker)

            # Text
            text_marker = Marker()
            text_marker.header.frame_id = 'map'
            text_marker.ns = '/frasier_visualization'
            text_marker.id = id
            text_marker.type = text_marker.TEXT_VIEW_FACING
            text_marker.text = location
            text_marker.scale.z = 0.25
            text_marker.pose.position.x = pose['x']
            text_marker.pose.position.y = pose['y']
            text_marker.pose.position.z = 0.2
            text_marker.color.a = 1.0
            text_list.append(text_marker)

        self.arrow_marker_array.markers = arrow_list
        self.text_marker_array.markers = text_list

    def publish_markers(self):
        rospy.loginfo("Waypoint Visualizer: Publishing markers")
        while not rospy.is_shutdown():
            self.waypoint_pub.publish(self.arrow_marker_array)
            self.text_pub.publish(self.text_marker_array)
            rospy.Rate(10).sleep()


if __name__ == '__main__':
    rospy.init_node('waypoint_visualizer')
    signal.signal(signal.SIGINT, signal_handler)
    wv = WaypointVisualizer()
    wv.create_marker_array()
    try:
        wv.publish_markers()
    except KeyboardInterrupt:
        rospy.signal_shutdown("Ctrl-c")