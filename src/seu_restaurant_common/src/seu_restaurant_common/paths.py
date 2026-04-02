import os

import rospkg


def resolve_package_path(package_name, relative_path=""):
    package_root = rospkg.RosPack().get_path(package_name)
    if not relative_path:
        return package_root
    return os.path.join(package_root, relative_path)


def resolve_config_path(package_name, config_name):
    return resolve_package_path(package_name, os.path.join("config", config_name))
