#!/usr/bin/env python3
import json
import os
import re
import uuid

import requests
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
    "six": 6,
}


CN_NUMBER_CHARS = {
    "一": 1,
    "二": 2,
    "两": 2,
    "三": 3,
    "四": 4,
    "五": 5,
    "六": 6,
}


class LLMFallbackParseOrderNode:
    def __init__(self):
        menu_cfg = load_package_config("seu_restaurant_common", "menu_items.yaml", rosparam_key="~menu_config")
        self.menu_items = menu_cfg.get("restaurant", {}).get("menu_items", {})
        self.service_name = get_service_name("llm", "parse_order", private_param="~parse_order_service_name")

        llm_cfg = rospy.get_param("/restaurant/llm", {})
        self.enable_llm = bool(rospy.get_param("~enable_llm", llm_cfg.get("enable_llm", True)))
        self.provider = str(rospy.get_param("~provider", llm_cfg.get("provider", "openai_compatible"))).strip().lower()
        self.model = str(rospy.get_param("~model", llm_cfg.get("model", "gpt-4o-mini"))).strip()
        self.timeout_sec = float(rospy.get_param("~timeout_sec", llm_cfg.get("timeout_sec", 20.0)))
        self.temperature = float(rospy.get_param("~temperature", llm_cfg.get("temperature", 0.0)))
        self.max_completion_tokens = int(rospy.get_param("~max_completion_tokens", llm_cfg.get("max_completion_tokens", 300)))
        self.base_url = self._resolve_base_url(rospy.get_param("~base_url", llm_cfg.get("base_url", "")))
        self.api_key, self.api_key_source = self._resolve_api_key(
            rospy.get_param("~api_key", llm_cfg.get("api_key", "")),
            llm_cfg.get("api_key_envs", []),
        )
        self._compiled_alias_patterns = self._compile_alias_patterns(self.menu_items)

        self.srv = rospy.Service(self.service_name, ParseOrder, self.handle_parse_order)
        rospy.loginfo(
            "[llm_fallback_parse_order_node] Service ready: %s llm_enabled=%s provider=%s model=%s base_url=%s api_key_present=%s",
            self.service_name,
            self.enable_llm,
            self.provider,
            self.model,
            self.base_url or "",
            bool(self.api_key),
        )
        rospy.loginfo(
            "[llm_fallback_parse_order_node] LLM config key_source=%s timeout=%.1fs temperature=%.2f max_completion_tokens=%d",
            self.api_key_source,
            self.timeout_sec,
            self.temperature,
            self.max_completion_tokens,
        )

    def handle_parse_order(self, req):
        raw_text = (req.raw_text or "").strip()
        if not raw_text:
            return ParseOrderResponse(success=False, order=OrderInfo(), message="empty raw_text")

        llm_error = ""
        if self.enable_llm:
            order_msg, llm_error = self._try_llm_parse(raw_text)
            if order_msg is not None:
                rospy.loginfo(
                    "[llm_fallback_parse_order_node] Parse path=llm items=%d raw_text='%s'",
                    len(order_msg.items),
                    raw_text,
                )
                return ParseOrderResponse(success=True, order=order_msg, message="llm parse success")
            if llm_error:
                rospy.logwarn("[llm_fallback_parse_order_node] LLM parse failed, fallback to rules: %s", llm_error)

        order_msg = self._rule_parse(raw_text)
        if order_msg is None:
            message = "no supported items recognized"
            if llm_error:
                message += "; llm_error={}".format(llm_error)
            return ParseOrderResponse(success=False, order=OrderInfo(), message=message)
        rospy.loginfo(
            "[llm_fallback_parse_order_node] Parse path=rule_fallback items=%d raw_text='%s'",
            len(order_msg.items),
            raw_text,
        )
        return ParseOrderResponse(success=True, order=order_msg, message="rule fallback parse success")

    def _try_llm_parse(self, raw_text):
        if self.provider != "openai_compatible":
            return None, "unsupported provider '{}'".format(self.provider)
        if not self.api_key:
            return None, "missing API key"
        if not self.base_url:
            return None, "missing base_url"

        menu_description = self._menu_description()
        system_prompt = (
            "You extract restaurant orders into strict JSON. "
            "Return JSON only with schema: "
            '{{"items":[{{"item_id":"<menu id>","quantity":<int>,"notes":""}}],"special_requests":""}}. '
            "Use only these menu ids and aliases: {}. "
            "If nothing is recognized, return {{\"items\":[],\"special_requests\":\"\"}}."
        ).format(menu_description)

        user_prompt = "Customer said: {}".format(raw_text)
        payload = {
            "model": self.model,
            "temperature": self.temperature,
            "max_tokens": self.max_completion_tokens,
            "messages": [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt},
            ],
        }

        base_url = self.base_url.rstrip("/")
        if base_url.endswith("/chat/completions"):
            url = base_url
        else:
            url = base_url + "/chat/completions"
        headers = {
            "Authorization": "Bearer {}".format(self.api_key),
            "Content-Type": "application/json",
        }

        try:
            response = requests.post(url, headers=headers, json=payload, timeout=self.timeout_sec)
            response.raise_for_status()
        except requests.RequestException as exc:
            return None, "request failed: {}".format(exc)

        try:
            response_json = response.json()
            content = response_json["choices"][0]["message"]["content"]
        except Exception as exc:
            return None, "invalid response shape: {}".format(exc)

        try:
            parsed = self._extract_json_object(content)
        except ValueError as exc:
            return None, "invalid JSON from llm: {}".format(exc)

        order_msg = self._order_from_struct(parsed)
        if order_msg is None or not order_msg.items:
            return None, "llm returned no recognized items"
        return order_msg, ""

    def _rule_parse(self, raw_text):
        text = raw_text.lower()
        items = []
        for item_id, cfg in self.menu_items.items():
            aliases = self._compiled_alias_patterns.get(item_id, [])
            total = 0
            for alias, pattern in aliases:
                for match in pattern.finditer(text):
                    total += self._parse_quantity_token(match.group(1))
                if self._contains_chinese(alias) and alias in raw_text:
                    total += self._parse_chinese_quantity(raw_text, alias)
            if total > 0:
                items.append(
                    OrderItem(
                        item_id=item_id,
                        display_name=cfg.get("display_name", item_id),
                        quantity=total,
                        notes="",
                    )
                )
        if not items:
            return None
        order_msg = OrderInfo()
        order_msg.order_id = "order_{}".format(uuid.uuid4().hex[:8])
        order_msg.items = items
        order_msg.special_requests = ""
        order_msg.confirmed = False
        return order_msg

    def _order_from_struct(self, data):
        raw_items = data.get("items", []) if isinstance(data, dict) else []
        if not isinstance(raw_items, list):
            return None

        items = []
        for entry in raw_items:
            if not isinstance(entry, dict):
                continue
            item_id = str(entry.get("item_id", "")).strip()
            if item_id not in self.menu_items:
                continue
            try:
                quantity = max(1, int(entry.get("quantity", 1)))
            except Exception:
                quantity = 1
            notes = str(entry.get("notes", "")).strip()
            items.append(
                OrderItem(
                    item_id=item_id,
                    display_name=self.menu_items[item_id].get("display_name", item_id),
                    quantity=quantity,
                    notes=notes,
                )
            )

        if not items:
            return None

        order_msg = OrderInfo()
        order_msg.order_id = "order_{}".format(uuid.uuid4().hex[:8])
        order_msg.items = items
        order_msg.special_requests = str(data.get("special_requests", "")).strip() if isinstance(data, dict) else ""
        order_msg.confirmed = False
        return order_msg

    def _menu_description(self):
        parts = []
        for item_id, cfg in self.menu_items.items():
            aliases = cfg.get("aliases", [])
            parts.append("{}: {}".format(item_id, ", ".join(aliases)))
        return " | ".join(parts)

    def _resolve_base_url(self, configured_value):
        configured_value = str(configured_value or "").strip()
        if configured_value:
            return configured_value.rstrip("/")
        for env_name in ("OPENAI_BASE_URL", "CLIPROXY_BASE_URL"):
            value = os.environ.get(env_name, "").strip()
            if value:
                return value.rstrip("/")
        return "https://api.openai.com/v1"

    def _resolve_api_key(self, configured_value, configured_envs):
        configured_value = str(configured_value or "").strip()
        if configured_value:
            return configured_value, "param:~api_key"
        env_names = []
        if isinstance(configured_envs, list):
            env_names.extend([str(item).strip() for item in configured_envs if str(item).strip()])
        elif configured_envs:
            env_names.append(str(configured_envs).strip())
        env_names.extend(["OPENAI_API_KEY", "CLIPROXY_API_KEY"])
        seen = set()
        for env_name in env_names:
            if env_name in seen:
                continue
            seen.add(env_name)
            value = os.environ.get(env_name, "").strip()
            if value:
                return value, "env:{}".format(env_name)
        return "", "missing"

    def _compile_alias_patterns(self, menu_items):
        compiled = {}
        for item_id, cfg in menu_items.items():
            patterns = []
            for alias in cfg.get("aliases", []):
                alias = str(alias)
                if self._contains_chinese(alias):
                    patterns.append((alias, re.compile(r"$^")))
                    continue
                quantity_part = r"(?:(\b\d+\b|\ba\b|\ban\b|\bone\b|\btwo\b|\bthree\b|\bfour\b|\bfive\b|\bsix\b)\s+)?"
                patterns.append((alias, re.compile(quantity_part + re.escape(alias.lower()) + r"\b")))
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
        pattern = re.compile(r"([一二两三四五六]?)\s*[份杯瓶个]?\s*" + re.escape(alias))
        for match in pattern.finditer(text):
            token = (match.group(1) or "").strip()
            count += CN_NUMBER_CHARS.get(token, 1)
        return count

    def _contains_chinese(self, text):
        return any("\u4e00" <= ch <= "\u9fff" for ch in text)

    def _extract_json_object(self, content):
        if not isinstance(content, str):
            raise ValueError("response content is not string")
        content = content.strip()
        try:
            return json.loads(content)
        except Exception:
            pass

        start = content.find("{")
        end = content.rfind("}")
        if start < 0 or end < 0 or end <= start:
            raise ValueError("no JSON object found")
        return json.loads(content[start : end + 1])


def main():
    rospy.init_node("llm_fallback_parse_order_node")
    LLMFallbackParseOrderNode()
    rospy.spin()


if __name__ == "__main__":
    main()
