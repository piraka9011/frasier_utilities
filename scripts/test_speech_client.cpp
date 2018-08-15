#include <frasier_utilities/speech_client.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_speech_client");
    ros::NodeHandle n;
    SpeechClient sc(n, true);
    sc.say("Hello!");
    return 0;
}
