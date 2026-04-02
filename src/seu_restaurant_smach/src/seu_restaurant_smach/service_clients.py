import copy

import rospy

from seu_restaurant_common.service_names import get_required_service_map, get_service_name
from seu_restaurant_msgs.srv import ListenText, ListenTextRequest, NavToPose, NavToPoseRequest, ParseOrder, ParseOrderRequest, SpeakText, SpeakTextRequest


def wait_for_required_services(timeout_sec):
    missing = []
    for _, service_name in get_required_service_map().items():
        try:
            rospy.wait_for_service(service_name, timeout=timeout_sec)
        except rospy.ROSException:
            missing.append(service_name)
    return missing


def speak_text(text):
    service_name = get_service_name("hri", "speak")
    proxy = rospy.ServiceProxy(service_name, SpeakText)
    req = SpeakTextRequest(text=text)
    return proxy(req)


def listen_text(prompt, timeout_sec):
    service_name = get_service_name("hri", "listen")
    proxy = rospy.ServiceProxy(service_name, ListenText)
    req = ListenTextRequest(prompt=prompt, timeout_sec=int(timeout_sec))
    return proxy(req)


def parse_order(raw_text):
    service_name = get_service_name("llm", "parse_order")
    proxy = rospy.ServiceProxy(service_name, ParseOrder)
    req = ParseOrderRequest(raw_text=raw_text)
    return proxy(req)


def nav_to_named_target(target_name):
    service_name = get_service_name("navigation", "nav_to_pose")
    proxy = rospy.ServiceProxy(service_name, NavToPose)
    req = NavToPoseRequest(target_name=target_name, use_named_target=True)
    return proxy(req)


def nav_to_pose(req):
    service_name = get_service_name("navigation", "nav_to_pose")
    proxy = rospy.ServiceProxy(service_name, NavToPose)
    return proxy(req)


def build_nav_request_for_customer(active_customer):
    req = NavToPoseRequest()

    dynamic_pose = None
    for attr_name in ("target_pose", "approach_pose", "customer_pose", "customer_target_pose", "position"):
        candidate = getattr(active_customer, attr_name, None)
        if candidate is not None and getattr(candidate.header, "frame_id", ""):
            dynamic_pose = copy.deepcopy(candidate)
            break

    if dynamic_pose is not None:
        req.use_named_target = False
        req.target_pose = dynamic_pose
        return req, {
            "mode": "dynamic_pose",
            "target_name": "",
            "frame_id": dynamic_pose.header.frame_id,
            "x": dynamic_pose.pose.position.x,
            "y": dynamic_pose.pose.position.y,
        }

    target_name = active_customer.pose_name or active_customer.table_id
    req.use_named_target = True
    req.target_name = target_name
    return req, {
        "mode": "named_target",
        "target_name": target_name,
        "frame_id": "",
        "x": None,
        "y": None,
    }
