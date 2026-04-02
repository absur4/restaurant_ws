import rospy

from seu_restaurant_smach.alignment import get_alignment_helper
from seu_restaurant_smach.states.base import MockRestaurantState


class AlignAtBarState(MockRestaurantState):
    def __init__(self):
        super(AlignAtBarState, self).__init__("ALIGN_AT_BAR")

    def run(self, task_context):
        mode = rospy.get_param("/restaurant/smach/alignment/bar_mode", "center_person")
        timeout_sec = float(rospy.get_param("/restaurant/smach/alignment/bar_timeout_sec", 6.0))
        success, message = get_alignment_helper().align_bar(mode=mode, timeout_sec=timeout_sec)
        if not success:
            task_context.last_error = "align_at_bar failed: {}".format(message)
            rospy.logerr("[ALIGN_AT_BAR] mode=%s timeout=%.1fs result=%s", mode, timeout_sec, message)
            return "failure"
        rospy.loginfo("[ALIGN_AT_BAR] mode=%s result=%s", mode, message)
        return "success"
