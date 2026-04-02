import rospy

from seu_restaurant_smach.service_clients import build_nav_request_for_customer, nav_to_pose
from seu_restaurant_smach.states.base import MockRestaurantState


class NavigateToCustomerState(MockRestaurantState):
    def __init__(self):
        super(NavigateToCustomerState, self).__init__("NAVIGATE_TO_CUSTOMER")

    def run(self, task_context):
        active_customer = task_context.current_customer or task_context.selected_customer
        req, target_meta = build_nav_request_for_customer(active_customer)
        if target_meta["mode"] == "dynamic_pose":
            rospy.loginfo(
                "[NAVIGATE_TO_CUSTOMER] customer_id=%s using dynamic target_pose frame=%s x=%.3f y=%.3f",
                active_customer.customer_id,
                target_meta["frame_id"],
                target_meta["x"],
                target_meta["y"],
            )
        else:
            rospy.loginfo(
                "[NAVIGATE_TO_CUSTOMER] customer_id=%s using named target target_name=%s",
                active_customer.customer_id,
                target_meta["target_name"],
            )

        response = nav_to_pose(req)
        if not response.success:
            task_context.last_error = response.message
            rospy.logerr("[NAVIGATE_TO_CUSTOMER] %s (code=%d)", response.message, response.failure_code)
            return "failure"
        if task_context.customer_memory is not None and active_customer.customer_id:
            task_context.customer_memory.mark_serving(active_customer.customer_id)
            self.publish_customer_memory_debug(
                task_context.customer_memory,
                "customer_serving",
                {
                    "current_customer_id": active_customer.customer_id,
                    "current_table_id": active_customer.table_id,
                    "current_pose_name": active_customer.pose_name,
                    "nav_mode": target_meta["mode"],
                },
            )
        rospy.loginfo("[NAVIGATE_TO_CUSTOMER] %s", response.message)
        return "success"
