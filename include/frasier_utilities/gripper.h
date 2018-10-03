#ifndef FRASIER_UTILITIES_GRIPPER_H
#define FRASIER_UTILITIES_GRIPPER_H

#include <ros/ros.h>
#include <ros/package.h>

#include <actionlib/client/simple_action_client.h>
#include <tmc_control_msgs/GripperApplyEffortGoal.h>
#include <tmc_control_msgs/GripperApplyEffortAction.h>

#include <iostream>
#include <frasier_utilities/hsr_constants.h>

class Gripper
{
private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionClient<tmc_control_msgs::GripperApplyEffortAction> gripper_cli_;
    bool debug_;

public:
    Gripper(ros::NodeHandle n, bool debug);
    bool grasp(double effort);
    bool grab();
    bool release();

    tmc_control_msgs::GripperApplyEffortGoal gripper_goal_;

    const double CONTROLLER_TIMEOUT = 50.0;
};


#endif //FRASIER_UTILITIES_GRIPPER_H