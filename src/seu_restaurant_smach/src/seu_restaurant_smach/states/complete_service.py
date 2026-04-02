import rospy

from seu_restaurant_smach.states.base import MockRestaurantState


class CompleteServiceState(MockRestaurantState):
    def __init__(self):
        super(CompleteServiceState, self).__init__("COMPLETE_SERVICE")

    def run(self, task_context):
        active_customer = task_context.current_customer or task_context.selected_customer
        if task_context.customer_memory is not None and active_customer.customer_id:
            task_context.customer_memory.mark_served(active_customer.customer_id)
            self.publish_customer_memory_debug(
                task_context.customer_memory,
                "customer_served",
                {"current_customer_id": active_customer.customer_id, "current_table_id": active_customer.table_id, "current_pose_name": active_customer.pose_name},
            )
        rospy.loginfo("[COMPLETE_SERVICE] Mock complete for customer=%s", active_customer.customer_id)
        return "success"
