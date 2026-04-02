import json
import traceback

import rospy
import smach
from std_msgs.msg import String


class MockRestaurantState(smach.State):
    def __init__(self, state_name, success_outcome="success", extra_outcomes=None):
        outcomes = [success_outcome, "failure"]
        if extra_outcomes:
            outcomes.extend(extra_outcomes)
        super(MockRestaurantState, self).__init__(
            outcomes=outcomes,
            input_keys=["task_context"],
            output_keys=["task_context"],
        )
        self._state_name = state_name
        self._customer_memory_debug_pub = None

    def execute(self, userdata):
        try:
            task_context = userdata.task_context
            rospy.loginfo("[%s] Enter", self._state_name)
            task_context.status = self._state_name
            task_context.note("{} entered".format(self._state_name))
            outcome = self.run(task_context)
            userdata.task_context = task_context
            rospy.loginfo("[%s] Exit with outcome=%s", self._state_name, outcome)
            return outcome
        except Exception as exc:
            rospy.logerr("[%s] Exception: %s", self._state_name, exc)
            rospy.logdebug(traceback.format_exc())
            if hasattr(userdata, "task_context") and userdata.task_context is not None:
                userdata.task_context.last_error = "{}: {}".format(self._state_name, exc)
                userdata.task_context.note(userdata.task_context.last_error)
            return "failure"

    def run(self, task_context):
        raise NotImplementedError

    def publish_customer_memory_debug(self, manager, reason, extra_payload=None):
        if manager is None:
            return
        if self._customer_memory_debug_pub is None:
            debug_topic = rospy.get_param("/restaurant/perception/customer_memory_debug_topic", "/restaurant/customer_memory_debug")
            self._customer_memory_debug_pub = rospy.Publisher(debug_topic, String, queue_size=10, latch=True)
        payload = manager.to_debug_dict()
        payload["reason"] = reason
        if extra_payload:
            payload.update(extra_payload)
        self._customer_memory_debug_pub.publish(String(data=json.dumps(payload, ensure_ascii=True, sort_keys=True)))
