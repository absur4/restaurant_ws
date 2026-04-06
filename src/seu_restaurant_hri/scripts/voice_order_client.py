#!/usr/bin/env python3
import sys

import rospy

from seu_restaurant_common.service_names import get_service_name
from seu_restaurant_msgs.srv import ListenText, ListenTextRequest, ParseOrder, ParseOrderRequest, SpeakText, SpeakTextRequest


class VoiceOrderClient:
    def __init__(self):
        self.speak_service_name = get_service_name("hri", "speak")
        self.listen_service_name = get_service_name("hri", "listen")
        self.parse_service_name = get_service_name("llm", "parse_order")

        self.service_wait_timeout_sec = float(rospy.get_param("~service_wait_timeout_sec", 5.0))
        self.listen_timeout_sec = int(
            rospy.get_param(
                "~listen_timeout_sec",
                rospy.get_param("/restaurant/smach/take_order/listen_timeout_sec", 8),
            )
        )
        self.max_attempts = int(
            rospy.get_param(
                "~max_attempts",
                int(rospy.get_param("/restaurant/smach/take_order/max_retries", 2)) + 1,
            )
        )
        self.welcome_prompt = rospy.get_param(
            "~welcome_prompt",
            rospy.get_param("/restaurant/smach/take_order/welcome_prompt", "Welcome. What would you like to order?"),
        )
        self.retry_prompt = rospy.get_param(
            "~retry_prompt",
            rospy.get_param("/restaurant/smach/take_order/retry_prompt", "I did not understand the order. Please say it again."),
        )
        self.result_template = rospy.get_param(
            "~result_template",
            "I understood your order: {items}.",
        )
        self.failure_template = rospy.get_param(
            "~failure_template",
            "Sorry, I could not understand the order.",
        )
        self.speak_result = bool(rospy.get_param("~speak_result", True))
        self.speak_failure = bool(rospy.get_param("~speak_failure", True))

        self._wait_for_services()
        self.speak_proxy = rospy.ServiceProxy(self.speak_service_name, SpeakText)
        self.listen_proxy = rospy.ServiceProxy(self.listen_service_name, ListenText)
        self.parse_proxy = rospy.ServiceProxy(self.parse_service_name, ParseOrder)

    def run(self):
        for attempt in range(self.max_attempts):
            prompt = self.welcome_prompt if attempt == 0 else self.retry_prompt
            if not self._speak(prompt):
                return 1

            try:
                listen_resp = self.listen_proxy(ListenTextRequest(prompt=prompt, timeout_sec=self.listen_timeout_sec))
            except rospy.ServiceException as exc:
                rospy.logerr("[voice_order_client] Listen service call failed: %s", exc)
                return 1
            if not listen_resp.success:
                rospy.logwarn(
                    "[voice_order_client] Listen failed on attempt %d/%d: %s",
                    attempt + 1,
                    self.max_attempts,
                    listen_resp.message,
                )
                continue

            raw_text = (listen_resp.text or "").strip()
            rospy.loginfo("[voice_order_client] Heard: %s", raw_text)

            try:
                parse_resp = self.parse_proxy(ParseOrderRequest(raw_text=raw_text))
            except rospy.ServiceException as exc:
                rospy.logerr("[voice_order_client] Parse service call failed: %s", exc)
                return 1
            if not parse_resp.success:
                rospy.logwarn(
                    "[voice_order_client] Parse failed on attempt %d/%d: %s",
                    attempt + 1,
                    self.max_attempts,
                    parse_resp.message,
                )
                continue

            order = parse_resp.order
            items_text = self._format_items(order.items)
            print("raw_text:", raw_text)
            print("items:", items_text)
            if order.special_requests:
                print("special_requests:", order.special_requests)

            if self.speak_result:
                self._speak(self.result_template.format(items=items_text))
            return 0

        rospy.logerr("[voice_order_client] Failed to understand order after %d attempt(s)", self.max_attempts)
        if self.speak_failure:
            self._speak(self.failure_template)
        return 2

    def _wait_for_services(self):
        for service_name in (self.speak_service_name, self.listen_service_name, self.parse_service_name):
            rospy.loginfo("[voice_order_client] Waiting for service: %s", service_name)
            rospy.wait_for_service(service_name, timeout=self.service_wait_timeout_sec)

    def _speak(self, text):
        if not hasattr(self, "speak_proxy"):
            return False
        try:
            response = self.speak_proxy(SpeakTextRequest(text=text))
        except rospy.ServiceException as exc:
            rospy.logerr("[voice_order_client] Speak service call failed: %s", exc)
            return False
        if response is None:
            return False
        if not response.success:
            rospy.logerr("[voice_order_client] Speak failed: %s", response.message)
            return False
        rospy.loginfo("[voice_order_client] Spoke: %s", text)
        return True

    def _format_items(self, items):
        formatted_items = []
        for item in items:
            name = item.display_name or item.item_id or "unknown_item"
            quantity = int(item.quantity) if int(item.quantity) > 0 else 1
            entry = "{} x{}".format(name, quantity)
            if item.notes:
                entry = "{} ({})".format(entry, item.notes)
            formatted_items.append(entry)
        if not formatted_items:
            return "no supported items"
        return ", ".join(formatted_items)


def main():
    rospy.init_node("voice_order_client")
    client = VoiceOrderClient()
    raise SystemExit(client.run())


if __name__ == "__main__":
    main()
