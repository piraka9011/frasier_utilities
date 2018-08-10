#include "yaml-cpp/yaml.h"
#include <iostream>

#include <ros/package.h>
#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>

#include <frasier_utilities/hsr_constants.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "yaml_test");

    ros::NodeHandle nh_;
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
        arm_cli_(ARM_CLIENT_TOPIC, true);
    arm_cli_.waitForServer(ros::Duration(3.0));

    trajectory_msgs::JointTrajectory arm_traj;
    control_msgs::FollowJointTrajectoryGoal arm_goal;
    arm_traj.joint_names = ARM_JOINTS;
    arm_traj.points.resize(1);
    arm_traj.points[0].positions.resize(5);
    arm_traj.points[0].time_from_start = ros::Duration(5.0);
    arm_goal.trajectory = arm_traj;

    std::string path = ros::package::getPath("frasier_utilities") + "/config/arm_configs.yaml";
    YAML::Node arm_config = YAML::LoadFile(path);

    std::string top_grasp = "top_grasp";
    std::vector<double> v = arm_config[top_grasp].as<std::vector<double > >();
    for (int i = 0; i < v.size(); i++) {
        arm_traj.points[0].positions[i] = v[i];
    }
    arm_goal.trajectory = arm_traj;
    arm_cli_.sendGoal(arm_goal);
    arm_cli_.waitForResult(ros::Duration(10));
}