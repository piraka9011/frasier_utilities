#!/usr/bin/env python

import rospy
from frasier_utilities import arm
from frasier_utilities.srv import StoreArmPosition, StoreArmPositionResponse


class StoreArmPositionService:
    def __init__(self):
        self.arm = arm.Arm()
        self.service = rospy.Service('/frasier_utilities/store_arm_position', StoreArmPosition, self.service_cb)

    def service_cb(self, req):
        result = self.arm.save_position(req.position_name)
        return StoreArmPositionResponse(result)


if __name__ == '__main__':
    rospy.init_node('store_arm_position_server')
    s = StoreArmPositionService()
    rospy.loginfo("Store Arm Position server is running...")
    rospy.spin()