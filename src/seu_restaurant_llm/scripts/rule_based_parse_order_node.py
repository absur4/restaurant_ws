#!/usr/bin/env python3
import re
import uuid

import rospy

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
}


MENU_ALIASES = {
    "coffee": ["coffee", "coffees", "咖啡"],
    "cola": ["cola", "colas", "coke", "cokes", "可乐"],
    "water": ["water", "waters", "矿泉水"],
    "tea": ["tea", "teas", "茶"],
    "milk": ["milk", "milks", "牛奶"],
    "juice": ["juice", "juices", "果汁"],
    "noodles": ["noodles", "noodle", "面条"],
    "chips": ["chips", "chip", "薯片"],
    "sandwich": ["sandwich", "sandwiches", "三明治"],
    "hamburger": ["hamburger", "hamburgers", "burger", "burgers", "汉堡"],
}


CN_NUMBER_CHARS = {
    "一": 1,
    "二": 2,
    "两": 2,
    "三": 3,
    "四": 4,
}


class RuleBasedParseOrderNode:
    def __init__(self):
        self.service_name = get_service_name("llm", "parse_order", private_param="~parse_order_service_name")
        self._compiled_alias_patterns = self._compile_alias_patterns(MENU_ALIASES)
        self.srv = rospy.Service(self.service_name, ParseOrder, self.handle_parse_order)
        rospy.loginfo("[rule_based_parse_order_node] Service ready: %s", self.service_name)

    def handle_parse_order(self, req):
        raw_text = (req.raw_text or "").strip()
        if not raw_text:
            return ParseOrderResponse(success=False, order=OrderInfo(), message="empty raw_text")

        items = self._parse_items(raw_text)
        if not items:
            return ParseOrderResponse(success=False, order=OrderInfo(), message="no supported items recognized")

        order_msg = OrderInfo()
        order_msg.order_id = "order_{}".format(uuid.uuid4().hex[:8])
        order_msg.items = items
        order_msg.special_requests = ""
        order_msg.confirmed = False
        rospy.loginfo(
            "[rule_based_parse_order_node] Parsed %d item(s) from '%s'",
            len(order_msg.items),
            raw_text,
        )
        return ParseOrderResponse(success=True, order=order_msg, message="parse success")

    def _parse_items(self, raw_text):
        text = raw_text.lower()
        quantity_by_item = {item_id: 0 for item_id in MENU_ALIASES.keys()}

        for item_id, alias_patterns in self._compiled_alias_patterns.items():
            total = 0
            for alias, pattern in alias_patterns:
                for match in pattern.finditer(text):
                    total += self._parse_quantity_token(match.group(1))
                if self._contains_chinese(alias) and alias in raw_text:
                    total += self._parse_chinese_quantity(raw_text, alias)
            quantity_by_item[item_id] += total

        items = []
        for item_id in MENU_ALIASES.keys():
            qty = quantity_by_item[item_id]
            if qty > 0:
                items.append(OrderItem(item_id=item_id, display_name=item_id, quantity=qty, notes=""))
        return items

    def _compile_alias_patterns(self, aliases_map):
        compiled = {}
        for item_id, aliases in aliases_map.items():
            patterns = []
            for alias in aliases:
                if self._contains_chinese(alias):
                    patterns.append((alias, re.compile(r"$^")))
                    continue
                quantity_part = r"(?:(\b\d+\b|\ba\b|\ban\b|\bone\b|\btwo\b|\bthree\b|\bfour\b)\s+)?"
                pattern = re.compile(quantity_part + re.escape(alias) + r"\b")
                patterns.append((alias, pattern))
            compiled[item_id] = patterns
        return compiled

    def _parse_quantity_token(self, token):
        if not token:
            return 1
        token = token.strip().lower()
        if token.isdigit():
            return max(1, int(token))
        return NUMBER_WORDS.get(token, 1)

    def _parse_chinese_quantity(self, text, alias):
        count = 0
        pattern = re.compile(r"([一二两三四]?)\s*[份杯瓶个]?\s*" + re.escape(alias))
        for match in pattern.finditer(text):
            token = (match.group(1) or "").strip()
            count += CN_NUMBER_CHARS.get(token, 1)
        return count

    def _contains_chinese(self, text):
        return any("\u4e00" <= ch <= "\u9fff" for ch in text)


def main():
    rospy.init_node("rule_based_parse_order_node")
    RuleBasedParseOrderNode()
    rospy.spin()


if __name__ == "__main__":
    main()
