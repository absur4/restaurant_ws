import copy
import threading

import rospy

from seu_restaurant_msgs.msg import CustomerCall
from seu_restaurant_smach.states.base import MockRestaurantState


class WaitForWaveEventsState(MockRestaurantState):
    def __init__(self):
        super(WaitForWaveEventsState, self).__init__("WAIT_FOR_WAVE_EVENTS", extra_outcomes=["scan"])
        self._lock = threading.Lock()
        self._pending_events = []
        self._subscriber = None

    def run(self, task_context):
        self._ensure_interfaces()
        manager = task_context.customer_memory
        if manager is None:
            raise RuntimeError("customer memory is not initialized")

        scan_window_sec = float(rospy.get_param("/restaurant/smach/scan_window_sec", 2.0))
        processed = self._consume_pending_events(manager)

        if manager.has_waiting_customers():
            self._publish_debug(manager, processed, "memory_already_has_waiting")
            rospy.loginfo("[WAIT_FOR_WAVE_EVENTS] Reusing %d waiting customer(s) already in memory", len(manager.get_waiting_customers()))
            return "success"

        rospy.loginfo("[WAIT_FOR_WAVE_EVENTS] Collecting wave events for %.2fs", scan_window_sec)
        deadline = rospy.Time.now() + rospy.Duration(scan_window_sec)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            processed += self._consume_pending_events(manager)
            rospy.sleep(0.05)

        processed += self._consume_pending_events(manager)
        manager.expire_old_entries()
        waiting_customers = manager.get_waiting_customers()
        self._publish_debug(manager, processed, "scan_complete")

        if waiting_customers:
            rospy.loginfo(
                "[WAIT_FOR_WAVE_EVENTS] Collected %d event(s), waiting queue=%s",
                processed,
                [entry.customer_id for entry in waiting_customers],
            )
            return "success"

        rospy.loginfo("[WAIT_FOR_WAVE_EVENTS] No customer wave detected within %.2fs", scan_window_sec)
        return "scan"

    def _ensure_interfaces(self):
        if self._subscriber is None:
            topic = rospy.get_param("/restaurant/perception/customer_call_topic", "/restaurant/customer_call")
            self._subscriber = rospy.Subscriber(topic, CustomerCall, self._customer_call_callback, queue_size=20)
        if self._subscriber is None:
            raise RuntimeError("failed to initialize subscriber")

    def _consume_pending_events(self, manager):
        pending = []
        with self._lock:
            if self._pending_events:
                pending = self._pending_events
                self._pending_events = []
        for event in pending:
            if event.source and event.source != "wave":
                rospy.logdebug("[WAIT_FOR_WAVE_EVENTS] Ignore non-wave event source=%s customer_id=%s", event.source, event.customer_id)
                continue
            manager.add_or_update_call(event)
        return len(pending)

    def _publish_debug(self, manager, processed_count, reason):
        self.publish_customer_memory_debug(manager, reason, {"processed_count": processed_count})

    def _customer_call_callback(self, msg):
        with self._lock:
            self._pending_events.append(copy.deepcopy(msg))
