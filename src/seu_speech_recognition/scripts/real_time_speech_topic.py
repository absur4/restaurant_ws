#!/usr/bin/env python3
import rospy
from std_msgs.msg import String


def main():
    rospy.init_node("real_time_speech_recognition_topic")
    pub = rospy.Publisher("/real_time_speech_recognition_topic", String, queue_size=10)
    rate = rospy.Rate(2)
    last_sent = ""
    while not rospy.is_shutdown():
        enabled = rospy.get_param("real_time_speech_recognition_enable", False)
        mock_text = rospy.get_param("~mock_text", "")
        if enabled and mock_text and mock_text != last_sent:
            pub.publish(String(data=mock_text))
            last_sent = mock_text
        rate.sleep()


if __name__ == "__main__":
    main()
