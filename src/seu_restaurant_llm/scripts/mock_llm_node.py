#!/usr/bin/env python3
import re
import uuid

import rospy

from seu_restaurant_common.config_loader import load_package_config
from seu_restaurant_common.service_names import get_service_name
from seu_restaurant_msgs.msg import OrderInfo, OrderItem
from seu_restaurant_msgs.srv import ParseOrder, ParseOrderResponse


NUMBER_WORDS = {
    "a": 1,
    "an": 1,
    "one": 1,
    "two": 2,
    "three": 3,
    "four": 4,
    "five": 5,
}


class MockLLMNode:
    def __init__(self):
        menu_cfg = load_package_config("seu_restaurant_common", "menu_items.yaml", rosparam_key="~menu_config")
        self.menu_items = menu_cfg.get("restaurant", {}).get("menu_items", {})
        service_name = get_service_name("llm", "parse_order", private_param="~parse_order_service_name")
        self.srv = rospy.Service(service_name, ParseOrder, self.handle_parse_order)
        rospy.loginfo("[mock_llm_node] Service ready: %s with %d menu items", service_name, len(self.menu_items))

    def handle_parse_order(self, req):
        text = (req.raw_text or "").strip().lower()
        if not text:
            return ParseOrderResponse(success=False, order=OrderInfo(), message="empty order text")
        parsed = self._parse_items(text)
        if not parsed:
            return ParseOrderResponse(success=False, order=OrderInfo(), message="no menu items recognized")
        order_msg = OrderInfo()
        order_msg.order_id = "order_{}".format(uuid.uuid4().hex[:8])
        order_msg.special_requests = self._extract_special_requests(text)
        order_msg.confirmed = False
        order_msg.items = parsed
        rospy.loginfo("[mock_llm_node] Parsed %d order items from '%s'", len(parsed), req.raw_text)
        return ParseOrderResponse(success=True, order=order_msg, message="parse success")

    def _parse_items(self, text):
        results = []
        for item_id, cfg in self.menu_items.items():
            aliases = cfg.get("aliases", [])
            display_name = cfg.get("display_name", item_id)
            total_qty = 0
            for alias in aliases:
                pattern = re.compile(r'(?:(\b\d+\b|\ba\b|\ban\b|\bone\b|\btwo\b|\bthree\b|\bfour\b|\bfive\b)\s+)?' + re.escape(alias.lower()) + r'\b')
                for match in pattern.finditer(text):
                    qty_token = match.group(1)
                    qty = self._parse_quantity(qty_token)
                    total_qty += qty
            if total_qty > 0:
                results.append(OrderItem(item_id=item_id, display_name=display_name, quantity=total_qty, notes=""))
        return results

    def _parse_quantity(self, token):
        if not token:
            return 1
        token = token.lower()
        if token.isdigit():
            return int(token)
        return NUMBER_WORDS.get(token, 1)

    def _extract_special_requests(self, text):
        if "no sugar" in text:
            return "no sugar"
        if "ice" in text:
            return "with ice"
        return ""


def main():
    rospy.init_node("mock_llm_node")
    MockLLMNode()
    rospy.spin()


if __name__ == "__main__":
    main()
