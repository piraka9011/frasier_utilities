#ifndef HSR_CONSTANTS_H
#define HSR_CONSTANTS_H

// Arm
const std::string ARM_CLIENT_TOPIC =
    "/hsrb/arm_trajectory_controller/follow_joint_trajectory";
const std::vector<std::string> ARM_JOINTS = 
    {"arm_lift_joint", "arm_flex_joint", "arm_roll_joint", 
     "wrist_flex_joint", "wrist_roll_joint"};
// Head
const std::string HEAD_CLIENT_TOPIC = "/hsrb/head_trajectory_controller/follow_joint_trajectory";
const std::string HEAD_STATE_TOPIC = "/hsrb/head_trajectory_controller/state";
const std::vector<std::string> HEAD_JOINTS = {"head_pan_joint", "head_pan_joint"};
const double FULL_PAN = 1.16;
const double FULL_TILT = 0.9;
// Speech
const std::string SPEECH_TOPIC = "/talk_request";
const std::string SPEECH_CLIENT_TOPIC = "/talk_request_action";
// Base
const std::string MOVE_BASE_TOPIC = "/move_base/move";
const std::string OMNI_BASE_CLIENT_TOPIC = "/hsrb/omni_base_controller/follow_joint_trajectory";
const std::vector<std::string> OMNI_BASE_JOINTS = {"odom_x", "odom_y", "odom_t"};
// Gripper
const std::string GRIPPER_CLIENT_TOPIC = "/hsrb/gripper_controller/grasp";
#endif //HSR_CONSTANTS_H
