#!/usr/bin/env python3
import rospy
import smach_ros

from seu_restaurant_smach.state_machine import build_state_machine


def main():
    rospy.init_node("restaurant_smach")
    sm = build_state_machine()
    introspection_enabled = rospy.get_param("~enable_smach_viewer", False)
    introspection_server = None
    if introspection_enabled:
        introspection_server = smach_ros.IntrospectionServer("restaurant_smach_viewer", sm, "/RESTAURANT_SMACH")
        introspection_server.start()
    outcome = sm.execute()
    rospy.loginfo("[restaurant_smach] Finished with outcome=%s", outcome)
    if introspection_server is not None:
        introspection_server.stop()


if __name__ == "__main__":
    main()
