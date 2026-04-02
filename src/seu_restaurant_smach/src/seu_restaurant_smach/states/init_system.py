import rospy

from seu_restaurant_common.config_loader import load_package_config
from seu_restaurant_common.customer_memory import CustomerMemoryManager
from seu_restaurant_common.service_names import get_required_service_map
from seu_restaurant_smach.service_clients import wait_for_required_services
from seu_restaurant_smach.states.base import MockRestaurantState


class InitSystemState(MockRestaurantState):
    def __init__(self):
        super(InitSystemState, self).__init__("INIT_SYSTEM")

    def run(self, task_context):
        defaults_cfg = load_package_config("seu_restaurant_common", "restaurant_defaults.yaml", rosparam_key="~defaults_config")
        memory_cfg = load_package_config("seu_restaurant_common", "customer_memory.yaml", rosparam_key="~customer_memory_config")
        menu_cfg = load_package_config("seu_restaurant_common", "menu_items.yaml", rosparam_key="~menu_config")
        services_cfg = load_package_config("seu_restaurant_common", "service_names.yaml", rosparam_key="~service_names_config")
        nav_cfg = load_package_config("seu_restaurant_navigation", "navigation_targets.yaml", rosparam_key="~navigation_targets_config")
        restaurant_cfg = defaults_cfg.get("restaurant", {})
        customer_memory_cfg = memory_cfg.get("restaurant", {}).get("customer_memory", {})
        task_context.available_tables = restaurant_cfg.get("tables", {})
        task_context.menu = menu_cfg.get("restaurant", {}).get("menu_items", {})
        task_context.service_names = services_cfg.get("restaurant", {}).get("services", {})
        task_context.navigation_targets = nav_cfg.get("restaurant", {}).get("navigation_targets", {})
        task_context.bar_pose_name = restaurant_cfg.get("bar", {}).get("pose_name", "bar_pose")
        task_context.customer_memory = CustomerMemoryManager(config=customer_memory_cfg)
        task_context.max_cycles = int(rospy.get_param("~max_cycles", 1))
        timeout_sec = float(rospy.get_param("/restaurant/smach/service_wait_timeout_sec", 3.0))
        missing = wait_for_required_services(timeout_sec)
        if missing:
            task_context.last_error = "missing services: {}".format(", ".join(missing))
            rospy.logerr("[INIT_SYSTEM] %s", task_context.last_error)
            return "failure"
        rospy.loginfo("[INIT_SYSTEM] Loaded %d tables, %d menu items, %d navigation targets", len(task_context.available_tables), len(task_context.menu), len(task_context.navigation_targets))
        rospy.loginfo("[INIT_SYSTEM] Customer memory policy=%s expire=%.2fs duplicate_distance=%.2f duplicate_time=%.2fs", task_context.customer_memory.selection_policy, task_context.customer_memory.memory_expire_sec, task_context.customer_memory.duplicate_distance_thresh, task_context.customer_memory.duplicate_time_thresh)
        rospy.loginfo("[INIT_SYSTEM] Required services available: %s", get_required_service_map())
        return "success"
