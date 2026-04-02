#!/usr/bin/env python3
import rospy

from seu_restaurant_common.config_loader import load_package_config
from seu_restaurant_common.service_names import get_service_name
from seu_restaurant_msgs.srv import ListenText, ListenTextResponse, SpeakText, SpeakTextResponse


class MockHRINode:
    def __init__(self):
        config = load_package_config("seu_restaurant_common", "mock_dialog.yaml", rosparam_key="~mock_dialog_config")
        hri_cfg = config.get("restaurant", {}).get("hri", {}).get("mock", {})
        self.default_timeout = int(rospy.get_param("~timeout_sec", hri_cfg.get("timeout_sec", 8)))
        self.retry_count = int(rospy.get_param("~retry_count", hri_cfg.get("retry_count", 2)))
        self.default_text = rospy.get_param("~default_text", hri_cfg.get("default_text", "one coffee"))
        self.prompt_rules = hri_cfg.get("prompt_rules", [])
        self.listen_response_override = rospy.get_param("~listen_response_text", "")
        speak_service_name = get_service_name("hri", "speak", private_param="~speak_service_name")
        listen_service_name = get_service_name("hri", "listen", private_param="~listen_service_name")
        self.speak_srv = rospy.Service(speak_service_name, SpeakText, self.handle_speak)
        self.listen_srv = rospy.Service(listen_service_name, ListenText, self.handle_listen)
        rospy.loginfo("[mock_hri_node] Services ready: speak=%s listen=%s", speak_service_name, listen_service_name)

    def handle_speak(self, req):
        rospy.loginfo("[mock_hri_node] SPEAK: %s", req.text)
        return SpeakTextResponse(success=True, message="mock speak success")

    def handle_listen(self, req):
        timeout_sec = int(req.timeout_sec) if req.timeout_sec > 0 else self.default_timeout
        prompt = (req.prompt or "").strip()
        response_text = self._pick_response(prompt)
        rospy.loginfo("[mock_hri_node] LISTEN prompt='%s' timeout=%s retry=%s -> '%s'", prompt, timeout_sec, self.retry_count, response_text)
        return ListenTextResponse(success=True, text=response_text, message="mock listen success")

    def _pick_response(self, prompt):
        if self.listen_response_override:
            return self.listen_response_override
        prompt_lower = prompt.lower()
        for rule in self.prompt_rules:
            keywords = [item.lower() for item in rule.get("contains", [])]
            if any(keyword in prompt_lower for keyword in keywords):
                return str(rule.get("response", self.default_text))
        return self.default_text


def main():
    rospy.init_node("mock_hri_node")
    MockHRINode()
    rospy.spin()


if __name__ == "__main__":
    main()
