DEFAULT_CONTEXT_NAMESPACE = "~task"
DEFAULT_FRAME_PARAMS = {
    "map_frame": "map",
    "base_frame": "base_link",
    "table_frame_prefix": "table_",
}
DEFAULT_TOPIC_PARAMS = {
    "customer_call_topic": "/restaurant/customer_call",
    "wave_event_topic": "/restaurant/perception/wave_event",
    "customer_memory_debug_topic": "/restaurant/customer_memory_debug",
    "order_result_topic": "/restaurant/order_result",
}
DEFAULT_SERVICE_NAMES = {
    "hri": {
        "speak": "/restaurant/hri/speak",
        "listen": "/restaurant/hri/listen",
    },
    "navigation": {
        "nav_to_pose": "/restaurant/navigation/nav_to_pose",
    },
    "llm": {
        "parse_order": "/restaurant/llm/parse_order",
    },
}
DEFAULT_MENU_PARAM = "/restaurant/menu_items"
DEFAULT_TABLES_PARAM = "/restaurant/tables"
DEFAULT_TARGETS_PARAM = "/restaurant/navigation_targets"
DEFAULT_SMACH_PARAM = "/restaurant/smach"
