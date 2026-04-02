#!/usr/bin/env python3
import rospy

from seu_restaurant_common.service_names import get_service_name
from seu_restaurant_hri.asr_bridge import LegacyASRBridge
from seu_restaurant_hri.tts_engine import speak
from seu_restaurant_msgs.srv import ListenText, ListenTextResponse, SpeakText, SpeakTextResponse


class RealHRINode:
    def __init__(self):
        self.asr_bridge = LegacyASRBridge()
        speak_service_name = get_service_name("hri", "speak", private_param="~speak_service_name")
        listen_service_name = get_service_name("hri", "listen", private_param="~listen_service_name")
        self.speak_srv = rospy.Service(speak_service_name, SpeakText, self.handle_speak)
        self.listen_srv = rospy.Service(listen_service_name, ListenText, self.handle_listen)
        rospy.loginfo("[real_hri_node] Services ready: speak=%s listen=%s", speak_service_name, listen_service_name)

    def handle_speak(self, req):
        success, message = speak(req.text)
        rospy.loginfo("[real_hri_node] SPEAK success=%s text=%s", success, req.text)
        return SpeakTextResponse(success=success, message=message)

    def handle_listen(self, req):
        success, text, message = self.asr_bridge.listen(prompt=req.prompt, timeout_sec=req.timeout_sec)
        rospy.loginfo(
            "[real_hri_node] LISTEN success=%s prompt='%s' timeout=%s -> '%s'",
            success,
            req.prompt,
            req.timeout_sec,
            text,
        )
        return ListenTextResponse(success=success, text=text, message=message)


def main():
    rospy.init_node("real_hri_node")
    RealHRINode()
    rospy.spin()


if __name__ == "__main__":
    main()
