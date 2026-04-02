#!/usr/bin/env python3
import rospy


def main():
    rospy.init_node("mock_manipulation_node")
    rospy.loginfo("[mock_manipulation_node] Placeholder manipulation node started")
    rospy.spin()


if __name__ == "__main__":
    main()
