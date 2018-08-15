#!/usr/bin/env python
# ROS
import rospy
import actionlib

# HSR
from tmc_msgs.msg import Voice, TalkRequestAction, TalkRequestGoal
from hsr_constants import *


class SpeechClient(object):
    def __init__(self):
        # Voice Msg
        self.voice_pub = rospy.Publisher(SPEECH_TOPIC, Voice, queue_size=10)
        self._speech_cli = actionlib.SimpleActionClient(SPEECH_CLIENT_TOPIC, TalkRequestAction)
        speech_is_running =  self._speech_cli.wait_for_server(rospy.Duration(2))

        if speech_is_running:
            print "SPEECH CLIENT: Speech initialized."
        else:
            print "SPEECH CLIENT: Speech is NOT initialized!"
        self.voice_msg = Voice()
        self.voice_msg.language = Voice.kEnglish
        self.voice_msg.queueing = True
        self.voice_msg.interrupting = True
        self.voice_msg.sentence = 'Start'
        self.goal = TalkRequestGoal()

    def sendCurrentGoal(self, blocking=True):
        self._speech_cli.send_goal(self.goal)
        if blocking:
            self._speech_cli.wait_for_result()

    def setSentence(self, text):
        self.voice_msg.sentence = text
        self.goal.data = self.voice_msg

    def say(self, text, blocking=True):
        self.setSentence(text)
        self.sendCurrentGoal(blocking)