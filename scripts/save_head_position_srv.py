#!/usr/bin/env python

import rospy
from frasier_utilities import head
from frasier_utilities.srv import StoreArmPosition, StoreArmPositionResponse


class StoreHeadPositionService:
    def __init__(self):
        self.head = head.Head()
        self.service = rospy.Service('/frasier_utilities/store_head_position', StoreArmPosition, self.service_cb)

    def service_cb(self, req):
        result = self.head.save_position(req.position_name)
        return StoreArmPositionResponse(result)


if __name__ == '__main__':
    rospy.init_node('store_head_position_server')
    s = StoreHeadPositionService()
    rospy.loginfo("Store Head Position server is running...")
    rospy.spin()
