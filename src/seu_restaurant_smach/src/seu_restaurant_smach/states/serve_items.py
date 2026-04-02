import rospy

from seu_restaurant_smach.service_clients import speak_text
from seu_restaurant_smach.states.base import MockRestaurantState


class ServeItemsState(MockRestaurantState):
    def __init__(self):
        super(ServeItemsState, self).__init__("SERVE_ITEMS")

    def run(self, task_context):
        template = rospy.get_param("/restaurant/smach/serve_items/template", "Here is your order. Please enjoy your meal.")
        response = speak_text(template)
        if not response.success:
            task_context.last_error = response.message
            rospy.logerr("[SERVE_ITEMS] Speak failed: %s", response.message)
            return "failure"
        rospy.loginfo("[SERVE_ITEMS] %s", response.message)
        return "success"
