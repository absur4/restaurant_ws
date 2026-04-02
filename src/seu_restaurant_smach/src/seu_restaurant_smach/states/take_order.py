import rospy

from seu_restaurant_common.restaurant_context import Order
from seu_restaurant_smach.service_clients import listen_text, parse_order, speak_text
from seu_restaurant_smach.states.base import MockRestaurantState


class TakeOrderState(MockRestaurantState):
    def __init__(self):
        super(TakeOrderState, self).__init__("TAKE_ORDER")

    def run(self, task_context):
        active_customer = task_context.current_customer or task_context.selected_customer
        max_retries = int(rospy.get_param("/restaurant/smach/take_order/max_retries", 2))
        listen_timeout_sec = int(rospy.get_param("/restaurant/smach/take_order/listen_timeout_sec", 8))
        welcome_prompt = rospy.get_param("/restaurant/smach/take_order/welcome_prompt", "Welcome. What would you like to order?")
        retry_prompt = rospy.get_param("/restaurant/smach/take_order/retry_prompt", "I did not understand the order. Please say it again.")
        for attempt in range(max_retries + 1):
            prompt = welcome_prompt if attempt == 0 else retry_prompt
            speak_resp = speak_text(prompt)
            if not speak_resp.success:
                task_context.last_error = speak_resp.message
                rospy.logerr("[TAKE_ORDER] Speak failed: %s", speak_resp.message)
                return "failure"
            listen_resp = listen_text(prompt, listen_timeout_sec)
            if not listen_resp.success:
                rospy.logwarn("[TAKE_ORDER] Listen failed on attempt %d: %s", attempt + 1, listen_resp.message)
                continue
            task_context.last_heard_text = listen_resp.text
            parse_resp = parse_order(listen_resp.text)
            if not parse_resp.success:
                rospy.logwarn("[TAKE_ORDER] Parse failed on attempt %d: %s", attempt + 1, parse_resp.message)
                continue
            task_context.current_order = Order.from_msg(parse_resp.order)
            task_context.current_order.customer_id = active_customer.customer_id
            task_context.current_order.table_id = active_customer.table_id
            task_context.current_order.raw_text = listen_resp.text
            task_context.current_order.confirmed = True
            rospy.loginfo("[TAKE_ORDER] Parsed order with %d item(s) from '%s'", len(task_context.current_order.items), listen_resp.text)
            return "success"
        task_context.last_error = "take order exceeded retry limit"
        rospy.logerr("[TAKE_ORDER] %s", task_context.last_error)
        return "failure"
