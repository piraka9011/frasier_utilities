#ifndef HSR_CONSTANTS_H
#define HSR_CONSTANTS_H

// Arm
const std::string ARM_CLIENT_TOPIC =
    "/hsrb/arm_trajectory_controller/follow_joint_trajectory";
const std::vector<std::string> ARM_JOINTS = 
    {"arm_lift_joint", "arm_flex_joint", "arm_roll_joint", 
     "wrist_flex_joint", "wrist_roll_joint"};
// Speech
const std::string SPEECH_TOPIC = "/talk_request";
const std::string SPEECH_CLIENT_TOPIC = "/talk_request_action";

#endif //HSR_CONSTANTS_H
