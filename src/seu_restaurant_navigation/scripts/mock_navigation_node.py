#!/usr/bin/env python3
import rospy

from seu_restaurant_common.service_names import get_service_name
from seu_restaurant_msgs.srv import NavToPose, NavToPoseResponse


class MockNavigationNode:
    def __init__(self):
        self.call_counts = {}
        self.always_success = bool(rospy.get_param("~always_success", rospy.get_param("/restaurant/navigation/mock/always_success", True)))
        self.fail_targets = set(rospy.get_param("~fail_targets", rospy.get_param("/restaurant/navigation/mock/fail_targets", [])))
        self.fail_first_n_calls = rospy.get_param("~fail_first_n_calls", rospy.get_param("/restaurant/navigation/mock/fail_first_n_calls", {}))
        self.default_failure_code = int(rospy.get_param("~default_failure_code", rospy.get_param("/restaurant/navigation/mock/default_failure_code", 100)))
        service_name = get_service_name("navigation", "nav_to_pose", private_param="~nav_service_name")
        self.srv = rospy.Service(service_name, NavToPose, self.handle_nav)
        rospy.loginfo("[mock_navigation_node] Service ready: %s", service_name)

    def handle_nav(self, req):
        target_key = req.target_name if req.use_named_target else req.target_pose.header.frame_id or "anonymous_pose"
        self.call_counts[target_key] = self.call_counts.get(target_key, 0) + 1
        call_index = self.call_counts[target_key]
        if self.always_success and target_key not in self.fail_targets and int(self.fail_first_n_calls.get(target_key, 0)) == 0:
            message = "mock navigation success to {}".format(target_key)
            rospy.loginfo("[mock_navigation_node] %s", message)
            return NavToPoseResponse(success=True, failure_code=0, message=message)
        if target_key in self.fail_targets:
            message = "configured failure for target {}".format(target_key)
            rospy.logwarn("[mock_navigation_node] %s", message)
            return NavToPoseResponse(success=False, failure_code=self.default_failure_code, message=message)
        fail_n = int(self.fail_first_n_calls.get(target_key, 0))
        if call_index <= fail_n:
            message = "temporary failure for target {} on call {} of {}".format(target_key, call_index, fail_n)
            rospy.logwarn("[mock_navigation_node] %s", message)
            return NavToPoseResponse(success=False, failure_code=self.default_failure_code + 1, message=message)
        message = "mock navigation success to {} after {} call(s)".format(target_key, call_index)
        rospy.loginfo("[mock_navigation_node] %s", message)
        return NavToPoseResponse(success=True, failure_code=0, message=message)


def main():
    rospy.init_node("mock_navigation_node")
    MockNavigationNode()
    rospy.spin()


if __name__ == "__main__":
    main()
