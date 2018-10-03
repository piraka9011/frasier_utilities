#include "frasier_utilities/gripper.h"

Gripper::Gripper(ros::NodeHandle n, bool debug = true) :
	nh_(n),
    gripper_cli_(GRIPPER_CLIENT_TOPIC, true),
	debug_(debug) 
{
	bool gripper_cli_running = gripper_cli_.waitForServer(ros::Duration(3.0));
	if (gripper_cli_running)
		std::cout << "GRIPPER CLIENT: Gripper controller initialized" << std::endl;
	else
		std::cout << "GRIPPER CLIENT: Gripper controller is NOT initialized!" << std::endl;

	gripper_goal_.effort = 0.0;
}

bool Gripper::grasp(double effort = 0.0) 
{
	gripper_goal_.effort = effort;
	gripper_cli_.sendGoal(gripper_goal_);
	gripper_cli_.waitForResult();

	if (gripper_cli_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		return true;
	else
		return false;
}

bool Gripper::release() 
{
	ROS_INFO("GRIPPER CLIENT: Releasing gripper.");
	return grasp(0.1);
}

bool Gripper::grab() 
{
	ROS_INFO("GRIPPER CLIENT: Closing gripper.");
	return grasp(-0.3);
}