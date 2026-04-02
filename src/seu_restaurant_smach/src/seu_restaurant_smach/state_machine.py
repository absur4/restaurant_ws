import rospy
import smach

from seu_restaurant_common.restaurant_context import TaskContext
from seu_restaurant_smach.states.align_at_bar import AlignAtBarState
from seu_restaurant_smach.states.align_at_table import AlignAtTableState
from seu_restaurant_smach.states.align_for_serving import AlignForServingState
from seu_restaurant_smach.states.acquire_items import AcquireItemsState
from seu_restaurant_smach.states.complete_service import CompleteServiceState
from seu_restaurant_smach.states.init_system import InitSystemState
from seu_restaurant_smach.states.loop_or_finish import LoopOrFinishState
from seu_restaurant_smach.states.navigate_back_to_customer import NavigateBackToCustomerState
from seu_restaurant_smach.states.navigate_to_bar import NavigateToBarState
from seu_restaurant_smach.states.navigate_to_customer import NavigateToCustomerState
from seu_restaurant_smach.states.report_order import ReportOrderState
from seu_restaurant_smach.states.select_next_customer import SelectNextCustomerState
from seu_restaurant_smach.states.serve_items import ServeItemsState
from seu_restaurant_smach.states.take_order import TakeOrderState
from seu_restaurant_smach.states.wait_for_wave_events import WaitForWaveEventsState


def build_state_machine():
    sm = smach.StateMachine(outcomes=["MISSION_FINISHED", "MISSION_ABORTED"])
    sm.userdata.task_context = TaskContext()

    with sm:
        smach.StateMachine.add("INIT_SYSTEM", InitSystemState(), transitions={"success": "WAIT_FOR_WAVE_EVENTS", "failure": "MISSION_ABORTED"})
        smach.StateMachine.add("WAIT_FOR_WAVE_EVENTS", WaitForWaveEventsState(), transitions={"success": "SELECT_NEXT_CUSTOMER", "scan": "WAIT_FOR_WAVE_EVENTS", "failure": "MISSION_ABORTED"})
        smach.StateMachine.add("SELECT_NEXT_CUSTOMER", SelectNextCustomerState(), transitions={"success": "NAVIGATE_TO_CUSTOMER", "empty": "WAIT_FOR_WAVE_EVENTS", "failure": "MISSION_ABORTED"})
        smach.StateMachine.add("NAVIGATE_TO_CUSTOMER", NavigateToCustomerState(), transitions={"success": "ALIGN_AT_TABLE", "failure": "MISSION_ABORTED"})
        smach.StateMachine.add("ALIGN_AT_TABLE", AlignAtTableState(), transitions={"success": "TAKE_ORDER", "failure": "MISSION_ABORTED"})
        smach.StateMachine.add("TAKE_ORDER", TakeOrderState(), transitions={"success": "NAVIGATE_TO_BAR", "failure": "MISSION_ABORTED"})
        smach.StateMachine.add("NAVIGATE_TO_BAR", NavigateToBarState(), transitions={"success": "ALIGN_AT_BAR", "failure": "MISSION_ABORTED"})
        smach.StateMachine.add("ALIGN_AT_BAR", AlignAtBarState(), transitions={"success": "REPORT_ORDER", "failure": "MISSION_ABORTED"})
        smach.StateMachine.add("REPORT_ORDER", ReportOrderState(), transitions={"success": "ACQUIRE_ITEMS", "failure": "MISSION_ABORTED"})
        smach.StateMachine.add("ACQUIRE_ITEMS", AcquireItemsState(), transitions={"success": "NAVIGATE_BACK_TO_CUSTOMER", "failure": "MISSION_ABORTED"})
        smach.StateMachine.add("NAVIGATE_BACK_TO_CUSTOMER", NavigateBackToCustomerState(), transitions={"success": "ALIGN_FOR_SERVING", "failure": "MISSION_ABORTED"})
        smach.StateMachine.add("ALIGN_FOR_SERVING", AlignForServingState(), transitions={"success": "SERVE_ITEMS", "failure": "MISSION_ABORTED"})
        smach.StateMachine.add("SERVE_ITEMS", ServeItemsState(), transitions={"success": "COMPLETE_SERVICE", "failure": "MISSION_ABORTED"})
        smach.StateMachine.add("COMPLETE_SERVICE", CompleteServiceState(), transitions={"success": "LOOP_OR_FINISH", "failure": "MISSION_ABORTED"})
        smach.StateMachine.add("LOOP_OR_FINISH", LoopOrFinishState(), transitions={"continue": "SELECT_NEXT_CUSTOMER", "wait": "WAIT_FOR_WAVE_EVENTS", "finish": "MISSION_FINISHED", "failure": "MISSION_ABORTED"})

    rospy.loginfo("Restaurant SMACH skeleton built successfully")
    return sm
