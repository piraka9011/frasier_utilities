#ifndef FRASIER_UTILITIES_SPEECH_CLIENT_H
#define FRASIER_UTILITIES_SPEECH_CLIENT_H
#include <ros/ros.h>
#include <ros/package.h>

#include <actionlib/client/simple_action_client.h>
#include <tmc_msgs/Voice.h>
#include <tmc_msgs/TalkRequestAction.h>

#include <iostream>
#include <frasier_utilities/hsr_constants.h>

class SpeechClient {
private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionClient<tmc_msgs::TalkRequestAction> speech_cli_;
    bool debug_;

public:
    SpeechClient(ros::NodeHandle n, bool debug = true);
    void sendCurrentGoal(bool wait = true);
    void setSentence(std::string sentence);
    void say(std::string sentence, bool wait = true);
    tmc_msgs::Voice voice_msg_;
    tmc_msgs::TalkRequestGoal voice_goal_;


};
#endif //FRASIER_UTILITIES_SPEECH_CLIENT_H
