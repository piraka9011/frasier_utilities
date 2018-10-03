#include <frasier_utilities/gripper.h>

int main(int argc, char** argv) 
{
	ros::init(argc, argv, "test_gripper_client");
    ros::NodeHandle n;
    Gripper gc(n, true);
    gc.grab();
    return 0;
}