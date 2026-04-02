import rospy

from seu_restaurant_smach.service_clients import nav_to_named_target
from seu_restaurant_smach.states.base import MockRestaurantState


class NavigateToBarState(MockRestaurantState):
    def __init__(self):
        super(NavigateToBarState, self).__init__("NAVIGATE_TO_BAR")

    def run(self, task_context):
        response = nav_to_named_target(task_context.bar_pose_name)
        if not response.success:
            task_context.last_error = response.message
            rospy.logerr("[NAVIGATE_TO_BAR] %s (code=%d)", response.message, response.failure_code)
            return "failure"
        rospy.loginfo("[NAVIGATE_TO_BAR] %s", response.message)
        return "success"
