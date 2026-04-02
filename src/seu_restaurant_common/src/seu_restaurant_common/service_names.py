import rospy

from .constants import DEFAULT_SERVICE_NAMES

SERVICE_ROOT_PARAM = "/restaurant/services"


def get_service_name(domain, name, private_param=""):
    if private_param and rospy.has_param(private_param):
        return rospy.get_param(private_param)
    param_name = "{}/{}/{}".format(SERVICE_ROOT_PARAM, domain, name)
    default = DEFAULT_SERVICE_NAMES.get(domain, {}).get(name, "")
    return rospy.get_param(param_name, default)


def get_required_service_map():
    return {
        "hri_speak": get_service_name("hri", "speak"),
        "hri_listen": get_service_name("hri", "listen"),
        "navigation_nav_to_pose": get_service_name("navigation", "nav_to_pose"),
        "llm_parse_order": get_service_name("llm", "parse_order"),
    }
