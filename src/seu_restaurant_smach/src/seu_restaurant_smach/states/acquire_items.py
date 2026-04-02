import rospy

from seu_restaurant_smach.states.base import MockRestaurantState


class AcquireItemsState(MockRestaurantState):
    def __init__(self):
        super(AcquireItemsState, self).__init__("ACQUIRE_ITEMS")

    def run(self, task_context):
        rospy.loginfo("[ACQUIRE_ITEMS] TODO: connect manipulation service. Keeping internal mock for now.")
        return "success"
