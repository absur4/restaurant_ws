import rospy

from seu_restaurant_common.restaurant_context import Customer
from seu_restaurant_smach.states.base import MockRestaurantState


class LoopOrFinishState(MockRestaurantState):
    def __init__(self):
        super(LoopOrFinishState, self).__init__("LOOP_OR_FINISH", success_outcome="finish", extra_outcomes=["continue", "wait"])

    def run(self, task_context):
        task_context.cycle_index += 1
        waiting_count = len(task_context.customer_memory.get_waiting_customers()) if task_context.customer_memory else 0
        task_context.current_customer = Customer()
        task_context.selected_customer = Customer()
        if task_context.should_finish:
            rospy.loginfo("[LOOP_OR_FINISH] Finishing after %d cycle(s)", task_context.cycle_index)
            return "finish"
        if waiting_count > 0:
            rospy.loginfo("[LOOP_OR_FINISH] Continue immediately with %d waiting customer(s)", waiting_count)
            return "continue"
        if task_context.max_cycles > 0 and task_context.cycle_index >= task_context.max_cycles:
            rospy.loginfo("[LOOP_OR_FINISH] Finishing after %d cycle(s)", task_context.cycle_index)
            return "finish"
        rospy.loginfo("[LOOP_OR_FINISH] No waiting customer in memory, returning to wave scan")
        return "wait"
