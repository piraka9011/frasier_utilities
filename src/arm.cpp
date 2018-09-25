#include "frasier_utilities/arm.h"

Arm::Arm(ros::NodeHandle n, bool debug = true) :
    nh_(n),
    arm_cli_(ARM_CLIENT_TOPIC, true),
    debug_(debug)
{
    bool arm_cli_running = arm_cli_.waitForServer(ros::Duration(3.0));
    if (arm_cli_running)
        std::cout << "ARM CLIENT: Arm controller initialized." << std::endl;
    else
        std::cout << "ARM CLIENT: Arm controller is NOT initialized!" << std::endl;

    // Yaml Setup
    path_ = ros::package::getPath("frasier_utilities") + "/config/arm_configs.yaml";
    arm_config_ = YAML::LoadFile(path_);

    // Arm traj. setup
    arm_traj.joint_names = ARM_JOINTS;
    arm_traj.points.resize(1);
    arm_traj.points[0].positions.resize(5);
    arm_traj.points[0].time_from_start = ros::Duration(5.0);
    arm_goal.trajectory = arm_traj;

}

void Arm::sendCurrentGoal(bool wait = true) {
    arm_cli_.sendGoal(arm_goal);
    if (wait)
        arm_cli_.waitForResult(ros::Duration(CONTROLLER_TIMEOUT));
}

void Arm::sendGoal(control_msgs::FollowJointTrajectoryGoal goal, bool wait = true) {
    arm_cli_.sendGoal(goal);
    if (wait)
        arm_cli_.waitForResult(ros::Duration(CONTROLLER_TIMEOUT));
}

void Arm::gotoPosition(std::vector<double> position) {
    for (int i = 0; i < position.size(); i++) {
        arm_traj.points[0].positions[i] = position[i];
    }
    arm_goal.trajectory = arm_traj;
    sendCurrentGoal();
}

void Arm::gotoKnownPosition(std::string position) {
    std::vector<double> positions = arm_config_[position].as<std::vector<double > >();
    for (int i = 0; i < positions.size(); i++) {
        arm_traj.points[0].positions[i] = positions[i];
    }

    arm_goal.trajectory = arm_traj;
    sendCurrentGoal();
}

