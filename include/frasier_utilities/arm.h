#ifndef FRASIER_UTILITIES_ARM_H
#define FRASIER_UTILITIES_ARM_H

#include <ros/ros.h>
#include <ros/package.h>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <geometry_msgs/Pose.h>

#include <yaml-cpp/yaml.h>
#include <iostream>
#include <frasier_utilities/hsr_constants.h>

class Arm
{
private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_cli_;
    geometry_msgs::Pose eef_target_pose_;
    bool debug_;
    std::string path_;
    YAML::Node arm_config_;

public:
    Arm(ros::NodeHandle n, bool debug);
    void gotoKnownPosition(std::string position);
    void gotoPosition(std::vector<double> position);
    void sendGoal(control_msgs::FollowJointTrajectoryGoal goal, bool wait);
    void sendCurrentGoal(bool wait);

    trajectory_msgs::JointTrajectory arm_traj;
    control_msgs::FollowJointTrajectoryGoal arm_goal;

    const double CONTROLLER_TIMEOUT = 50.0;
};


#endif //FRASIER_UTILITIES_ARM_H
