#!/usr/bin/env python

import rospy
from frasier_utilities.speech import Speech

try:
    rospy.init_node('speech_test')
    s = Speech()
    s.say('hello')
except KeyboardInterrupt:
    rospy.signal_shutdown("Bye")
