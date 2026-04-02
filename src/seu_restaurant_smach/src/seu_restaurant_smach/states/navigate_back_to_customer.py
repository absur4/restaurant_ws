import rospy

from seu_restaurant_smach.service_clients import build_nav_request_for_customer, nav_to_pose
from seu_restaurant_smach.states.base import MockRestaurantState


class NavigateBackToCustomerState(MockRestaurantState):
    def __init__(self):
        super(NavigateBackToCustomerState, self).__init__("NAVIGATE_BACK_TO_CUSTOMER")

    def run(self, task_context):
        active_customer = task_context.current_customer or task_context.selected_customer
        req, target_meta = build_nav_request_for_customer(active_customer)
        if target_meta["mode"] == "dynamic_pose":
            rospy.loginfo(
                "[NAVIGATE_BACK_TO_CUSTOMER] customer_id=%s using dynamic target_pose frame=%s x=%.3f y=%.3f",
                active_customer.customer_id,
                target_meta["frame_id"],
                target_meta["x"],
                target_meta["y"],
            )
        else:
            rospy.loginfo(
                "[NAVIGATE_BACK_TO_CUSTOMER] customer_id=%s using named target target_name=%s",
                active_customer.customer_id,
                target_meta["target_name"],
            )

        response = nav_to_pose(req)
        if not response.success:
            task_context.last_error = response.message
            rospy.logerr("[NAVIGATE_BACK_TO_CUSTOMER] %s (code=%d)", response.message, response.failure_code)
            return "failure"
        rospy.loginfo("[NAVIGATE_BACK_TO_CUSTOMER] %s", response.message)
        return "success"
