import rospy

from seu_restaurant_smach.service_clients import speak_text
from seu_restaurant_smach.states.base import MockRestaurantState


class ReportOrderState(MockRestaurantState):
    def __init__(self):
        super(ReportOrderState, self).__init__("REPORT_ORDER")

    def run(self, task_context):
        items = ", ".join(["{} x{}".format(item.display_name, item.quantity) for item in task_context.current_order.items])
        template = rospy.get_param("/restaurant/smach/report_order/template", "Reporting order for {customer_id}: {items}.")
        text = template.format(customer_id=task_context.selected_customer.customer_id, items=items)
        response = speak_text(text)
        if not response.success:
            task_context.last_error = response.message
            rospy.logerr("[REPORT_ORDER] Speak failed: %s", response.message)
            return "failure"
        rospy.loginfo("[REPORT_ORDER] %s", response.message)
        return "success"
