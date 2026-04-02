import rospy

from seu_restaurant_smach.alignment import get_alignment_helper
from seu_restaurant_smach.states.base import MockRestaurantState


class AlignForServingState(MockRestaurantState):
    def __init__(self):
        super(AlignForServingState, self).__init__("ALIGN_FOR_SERVING")

    def run(self, task_context):
        active_customer = task_context.current_customer or task_context.selected_customer
        timeout_sec = float(rospy.get_param("/restaurant/smach/alignment/serving_timeout_sec", 6.0))
        success, message = get_alignment_helper().align_customer(active_customer.customer_id, timeout_sec=timeout_sec)
        if not success:
            task_context.last_error = "align_for_serving failed: {}".format(message)
            rospy.logerr(
                "[ALIGN_FOR_SERVING] customer_id=%s timeout=%.1fs result=%s",
                active_customer.customer_id,
                timeout_sec,
                message,
            )
            return "failure"
        rospy.loginfo("[ALIGN_FOR_SERVING] customer_id=%s result=%s", active_customer.customer_id, message)
        return "success"
