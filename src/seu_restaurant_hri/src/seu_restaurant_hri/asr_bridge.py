import rospy
from std_msgs.msg import String

from seu_restaurant_common.config_loader import load_package_config

try:
    from seu_speech_recognition.srv import seu_speech_recognition, seu_speech_recognitionRequest  # type: ignore

    _HAS_LEGACY_ASR_SRV = True
except Exception:
    seu_speech_recognition = None
    seu_speech_recognitionRequest = None
    _HAS_LEGACY_ASR_SRV = False


class LegacyASRBridge:
    def __init__(self):
        config = load_package_config("seu_restaurant_common", "mock_dialog.yaml", rosparam_key="~mock_dialog_config")
        hri_cfg = config.get("restaurant", {}).get("hri", {}).get("mock", {})
        real_cfg = rospy.get_param("/restaurant/hri/real", {})
        self.default_timeout = int(rospy.get_param("~timeout_sec", hri_cfg.get("timeout_sec", 8)))
        self.default_task = rospy.get_param("~default_task", "command")
        self.default_text = rospy.get_param("~default_text", hri_cfg.get("default_text", "one coffee"))
        self.prompt_rules = hri_cfg.get("prompt_rules", [])
        self.mock_text = rospy.get_param("~mock_text", "")
        self.listen_response_override = rospy.get_param("~listen_response_text", "")
        self.use_legacy_asr_service = bool(
            rospy.get_param("~use_legacy_asr_service", real_cfg.get("use_legacy_asr_service", True))
        )
        self.allow_mock_fallback = bool(
            rospy.get_param("~allow_mock_fallback", real_cfg.get("allow_mock_fallback", False))
        )
        self.legacy_asr_service_name = rospy.get_param(
            "~legacy_asr_service_name", real_cfg.get("legacy_asr_service_name", "speech_recognition_service")
        )
        self.legacy_wait_timeout_sec = float(
            rospy.get_param("~legacy_wait_timeout_sec", real_cfg.get("legacy_wait_timeout_sec", 0.8))
        )
        self.publish_realtime_topic = bool(
            rospy.get_param("~publish_realtime_topic", real_cfg.get("publish_realtime_topic", False))
        )
        self.realtime_topic_name = rospy.get_param(
            "~realtime_topic_name", real_cfg.get("realtime_topic_name", "/real_time_speech_recognition_topic")
        )
        self._legacy_proxy = None
        self._last_published = ""
        self._publisher = None
        if self.publish_realtime_topic:
            self._publisher = rospy.Publisher(self.realtime_topic_name, String, queue_size=10)

    def listen(self, prompt="", timeout_sec=0):
        timeout_sec = int(timeout_sec) if int(timeout_sec) > 0 else self.default_timeout
        text, message = self._listen_from_legacy_service(prompt, timeout_sec)
        if text:
            self._publish_if_needed(text)
            return True, text, message

        if not self.allow_mock_fallback:
            if message:
                return False, "", message
            return False, "", "real ASR returned no valid text and mock fallback is disabled"

        text = self._resolve_text(prompt)
        if not text:
            if message:
                return False, "", message
            return False, "", "ASR backend not implemented; set ~mock_text or ~listen_response_text"
        self._publish_if_needed(text)
        fallback_message = "fallback ASR response (timeout={}s)".format(timeout_sec)
        if message:
            fallback_message += "; " + message
        return True, text, fallback_message

    def _listen_from_legacy_service(self, prompt, timeout_sec):
        if not self.use_legacy_asr_service:
            return "", ""
        if not _HAS_LEGACY_ASR_SRV:
            return "", "legacy ASR srv type not importable"
        try:
            if self._legacy_proxy is None:
                wait_timeout = min(float(timeout_sec), self.legacy_wait_timeout_sec)
                wait_timeout = max(wait_timeout, 0.1)
                rospy.wait_for_service(self.legacy_asr_service_name, timeout=wait_timeout)
                self._legacy_proxy = rospy.ServiceProxy(self.legacy_asr_service_name, seu_speech_recognition)
            req = seu_speech_recognitionRequest(enable=True, task=self._build_task(prompt))
            resp = self._legacy_proxy(req)
        except (rospy.ROSException, rospy.ServiceException) as exc:
            return "", "legacy ASR service unavailable: {}".format(exc)
        except Exception as exc:
            return "", "legacy ASR exception: {}".format(exc)

        text = (resp.word or "").strip()
        if resp.state and text:
            return text, "legacy ASR service response"
        err = (resp.errormsg or "").strip() or "legacy ASR returned empty text"
        return "", err

    def _build_task(self, prompt):
        prompt = (prompt or "").strip()
        return prompt if prompt else self.default_task

    def _resolve_text(self, prompt):
        if self.mock_text:
            return self.mock_text
        if self.listen_response_override:
            return self.listen_response_override
        prompt_lower = (prompt or "").lower()
        for rule in self.prompt_rules:
            keywords = [item.lower() for item in rule.get("contains", [])]
            if any(keyword in prompt_lower for keyword in keywords):
                return str(rule.get("response", self.default_text))
        return self.default_text

    def _publish_if_needed(self, text):
        if self._publisher is None or not text or text == self._last_published:
            return
        self._publisher.publish(String(data=text))
        self._last_published = text
