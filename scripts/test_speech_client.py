#!/usr/bin/env python

import rospy
from frasier_utilities.speech_client import SpeechClient

try:
    rospy.init_node('speech_test')
    s = SpeechClient()
    s.say('hello')
except KeyboardInterrupt:
    rospy.signal_shutdown("Bye")
