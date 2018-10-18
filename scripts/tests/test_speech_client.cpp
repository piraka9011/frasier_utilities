#include <frasier_utilities/speech.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_speech_client");
    ros::NodeHandle n;
    Speech sc(n, true);
    sc.say("Hello!");
    return 0;
}
