import copy

import rospy

from seu_restaurant_common.customer_memory import resolve_customer_table
from seu_restaurant_common.restaurant_context import Customer
from seu_restaurant_smach.states.base import MockRestaurantState


class SelectNextCustomerState(MockRestaurantState):
    def __init__(self):
        super(SelectNextCustomerState, self).__init__("SELECT_NEXT_CUSTOMER", extra_outcomes=["empty"])

    def run(self, task_context):
        manager = task_context.customer_memory
        if manager is None:
            raise RuntimeError("customer memory is not initialized")

        entry = manager.get_next_customer()
        if entry is None:
            rospy.loginfo("[SELECT_NEXT_CUSTOMER] No waiting customers in memory")
            return "empty"

        table_id, pose_name = resolve_customer_table(entry, task_context.available_tables, task_context.navigation_targets)
        entry.table_id = table_id or entry.table_id
        manager.mark_selected(entry.customer_id)

        customer = Customer(
            customer_id=entry.customer_id,
            table_id=entry.table_id,
            pose_name=pose_name or entry.table_id,
            notes="selected from customer memory",
            call_source=entry.source,
            confidence=entry.confidence,
            position=copy.deepcopy(entry.position),
            target_pose=copy.deepcopy(entry.position),
        )
        task_context.current_customer = customer
        task_context.selected_customer = customer
        self.publish_customer_memory_debug(
            manager,
            "customer_selected",
            {"current_customer_id": customer.customer_id, "current_table_id": customer.table_id, "current_pose_name": customer.pose_name},
        )
        rospy.loginfo(
            "[SELECT_NEXT_CUSTOMER] Selected customer_id=%s table_id=%s pose_name=%s target_frame=%s queue_remaining=%d",
            customer.customer_id,
            customer.table_id,
            customer.pose_name,
            customer.target_pose.header.frame_id if customer.target_pose is not None else "",
            max(0, len(manager.get_waiting_customers())),
        )
        return "success"
