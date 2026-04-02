import os

import rospy
import yaml

from .paths import resolve_config_path


def load_yaml_file(path):
    if not os.path.isfile(path):
        raise FileNotFoundError("Config file not found: {}".format(path))
    with open(path, "r") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise ValueError("Expected dict config in {}, got {}".format(path, type(data).__name__))
    return data


def load_package_config(package_name, config_name, rosparam_key=""):
    override = rospy.get_param(rosparam_key, "") if rosparam_key else ""
    config_path = override if override else resolve_config_path(package_name, config_name)
    return load_yaml_file(config_path)


def load_rosparam_dict(param_name, default=None):
    if default is None:
        default = {}
    value = rospy.get_param(param_name, default)
    if not isinstance(value, dict):
        raise ValueError("ROS param {} must be a dict".format(param_name))
    return value
