#!/usr/bin/env python
# ROS
import rospy
import actionlib

# HSR
from tmc_msgs.msg import Voice, TalkRequestAction, TalkRequestGoal
from hsr_constants import *


class Speech(object):
    def __init__(self):
        # Voice Msg
        self.voice_pub = rospy.Publisher(SPEECH_TOPIC, Voice, queue_size=10)
        self._speech_cli = actionlib.SimpleActionClient(SPEECH_CLIENT_TOPIC, TalkRequestAction)
        rospy.loginfo("SPEECH CLIENT: Waiting for speech action server...")
        speech_is_running = self._speech_cli.wait_for_server(rospy.Duration(2))

        if speech_is_running:
            rospy.loginfo("SPEECH CLIENT: Speech initialized.")
        else:
            rospy.loginfo("SPEECH CLIENT: Speech is NOT initialized!")
        self.voice_msg = Voice()
        self.voice_msg.language = Voice.kEnglish
        self.voice_msg.queueing = True
        self.voice_msg.interrupting = True
        self.voice_msg.sentence = 'Start'
        self.goal = TalkRequestGoal()

    def send_current_goal(self, blocking=True):
        self._speech_cli.send_goal(self.goal)
        if blocking:
            self._speech_cli.wait_for_result()

    def set_sentence(self, text):
        self.voice_msg.sentence = text
        self.goal.data = self.voice_msg

    def say(self, text, blocking=True):
        self.set_sentence(text)
        self.send_current_goal(blocking)
