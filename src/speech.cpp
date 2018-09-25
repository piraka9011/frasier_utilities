#include <frasier_utilities/speech.h>

Speech::Speech(ros::NodeHandle n, bool debug) :
	nh_(n),
	speech_cli_(SPEECH_CLIENT_TOPIC, true),
	debug_(debug)
{
	bool speech_cli_running = speech_cli_.waitForServer(ros::Duration(3.0));
	if (speech_cli_running)
		std::cout << "SPEECH CLIENT: Speech initialized." << std::endl;
	else
		std::cout << "SPEECH CLIENT: Speech is NOT initialized!" << std::endl;

	// Voice setup
	voice_msg_.language = tmc_msgs::Voice::kEnglish;
	voice_msg_.queueing = true;
	voice_msg_.interrupting = true;
	voice_msg_.sentence = "Start";
	voice_goal_.data = voice_msg_;
}

void Speech::sendCurrentGoal(bool wait) {
	speech_cli_.sendGoal(voice_goal_);
	if (wait)
		speech_cli_.waitForResult();
}

void Speech::setSentence(std::string sentence) {
	voice_msg_.sentence = sentence;
	voice_goal_.data = voice_msg_;
}

void Speech::say(std::string sentence, bool wait) {
	setSentence(sentence);
	sendCurrentGoal(wait);
}
