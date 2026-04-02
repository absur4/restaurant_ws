import json
import re
import threading

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String


_WAVE_CUSTOMER_ID_PATTERN = re.compile(r"wave_track_(\d+)$")


class YawAlignmentHelper:
    def __init__(self):
        base_param = "/restaurant/smach/alignment"
        self.wave_candidates_topic = rospy.get_param(base_param + "/wave_candidates_topic", "/restaurant/perception/wave_candidates")
        self.cmd_vel_topic = rospy.get_param(base_param + "/cmd_vel_topic", "/cmd_vel")
        self.control_hz = float(rospy.get_param(base_param + "/control_hz", 10.0))
        self.angular_kp = float(rospy.get_param(base_param + "/angular_kp", 1.8))
        self.max_ang_vel = float(rospy.get_param(base_param + "/max_ang_vel", 0.5))
        self.min_ang_vel = float(rospy.get_param(base_param + "/min_ang_vel", 0.08))
        self.aligned_pixel_thresh = float(rospy.get_param(base_param + "/aligned_pixel_thresh", 25.0))
        self.aligned_stable_cycles = max(1, int(rospy.get_param(base_param + "/aligned_stable_cycles", 2)))
        self.default_customer_timeout_sec = float(rospy.get_param(base_param + "/customer_timeout_sec", 8.0))
        self.default_bar_timeout_sec = float(rospy.get_param(base_param + "/bar_timeout_sec", 6.0))
        self.default_bar_mode = rospy.get_param(base_param + "/bar_mode", "center_person")
        self.allow_no_target_bypass = bool(rospy.get_param(base_param + "/allow_no_target_bypass", False))
        self.allow_no_target_bypass_in_mock_perception = bool(
            rospy.get_param(base_param + "/allow_no_target_bypass_in_mock_perception", True)
        )

        self._lock = threading.Lock()
        self._latest_candidates = {}
        self._candidate_sub = rospy.Subscriber(self.wave_candidates_topic, String, self._candidate_callback, queue_size=10)
        self._cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)

        rospy.loginfo(
            "[yaw_alignment] ready wave_candidates_topic=%s cmd_vel_topic=%s mode=%s",
            self.wave_candidates_topic,
            self.cmd_vel_topic,
            self.default_bar_mode,
        )

    def align_customer(self, customer_id, timeout_sec=None):
        timeout = float(timeout_sec) if timeout_sec is not None else self.default_customer_timeout_sec
        track_id = self._track_id_from_customer_id(customer_id)
        if track_id is None:
            rospy.logwarn(
                "[yaw_alignment] customer_id=%s does not map to wave_track_XX, fallback to center-person align",
                customer_id,
            )

            def selector(tracks):
                candidates = [track for track in tracks if str(track.get("state", "tentative")) != "lost"]
                if not candidates:
                    return None
                return min(candidates, key=lambda item: abs(self._center_u(item) - 0.5))

            return self._align_loop(
                selector=selector,
                timeout_sec=timeout,
                target_desc="customer_id={} fallback=center_person".format(customer_id),
            )

        def selector(tracks):
            for track in tracks:
                if int(track.get("track_id", -1)) == track_id:
                    return track
            return None

        return self._align_loop(
            selector=selector,
            timeout_sec=timeout,
            target_desc="customer_id={} track_id={}".format(customer_id, track_id),
        )

    def align_bar(self, mode="", timeout_sec=None):
        timeout = float(timeout_sec) if timeout_sec is not None else self.default_bar_timeout_sec
        bar_mode = mode or self.default_bar_mode

        def selector(tracks):
            candidates = [track for track in tracks if str(track.get("state", "tentative")) != "lost"]
            if not candidates:
                return None
            if bar_mode == "nearest_person":
                return max(candidates, key=self._track_area)
            if bar_mode == "highest_confidence":
                return max(candidates, key=lambda item: float(item.get("event_confidence", 0.0)))
            return min(candidates, key=lambda item: abs(self._center_u(item) - 0.5))

        return self._align_loop(
            selector=selector,
            timeout_sec=timeout,
            target_desc="bar_mode={}".format(bar_mode),
        )

    def _align_loop(self, selector, timeout_sec, target_desc):
        deadline = rospy.Time.now() + rospy.Duration(max(0.0, timeout_sec))
        stable_count = 0
        seen_target = False
        rate = rospy.Rate(max(1.0, self.control_hz))

        while not rospy.is_shutdown():
            if rospy.Time.now() >= deadline:
                self._publish_angular(0.0)
                if not seen_target:
                    if self._allow_no_target_bypass():
                        message = "no target visible, bypass"
                        rospy.logwarn("[yaw_alignment] %s %s", target_desc, message)
                        return True, message
                    message = "no target visible within {:.1f}s".format(timeout_sec)
                    rospy.logwarn("[yaw_alignment] %s %s", target_desc, message)
                    return False, message
                message = "timeout after {:.1f}s".format(timeout_sec)
                rospy.logwarn("[yaw_alignment] %s %s", target_desc, message)
                return False, message

            target_track = selector(self._get_tracks())
            if target_track is None:
                stable_count = 0
                self._publish_angular(0.0)
                rospy.logwarn_throttle(1.0, "[yaw_alignment] %s no target visible", target_desc)
                rate.sleep()
                continue

            seen_target = True
            image_width = max(1.0, float(self._image_width(target_track)))
            error_px = (self._center_u(target_track) - 0.5) * image_width
            rospy.loginfo_throttle(
                0.5,
                "[yaw_alignment] %s target_track_id=%d current_pixel_error=%.1f",
                target_desc,
                int(target_track.get("track_id", -1)),
                error_px,
            )

            if abs(error_px) <= self.aligned_pixel_thresh:
                stable_count += 1
                self._publish_angular(0.0)
                if stable_count >= self.aligned_stable_cycles:
                    message = "aligned"
                    rospy.loginfo(
                        "[yaw_alignment] %s target_track_id=%d aligned current_pixel_error=%.1f",
                        target_desc,
                        int(target_track.get("track_id", -1)),
                        error_px,
                    )
                    return True, message
            else:
                stable_count = 0
                cmd = max(-self.max_ang_vel, min(self.max_ang_vel, -self.angular_kp * (error_px / image_width)))
                if 0.0 < abs(cmd) < self.min_ang_vel:
                    cmd = self.min_ang_vel if cmd > 0.0 else -self.min_ang_vel
                self._publish_angular(cmd)
            rate.sleep()

        self._publish_angular(0.0)
        return False, "interrupted"

    def _candidate_callback(self, msg):
        try:
            payload = json.loads(msg.data)
        except ValueError:
            payload = {}
        with self._lock:
            self._latest_candidates = payload

    def _get_tracks(self):
        with self._lock:
            payload = dict(self._latest_candidates)
        tracks = payload.get("tracks", [])
        return tracks if isinstance(tracks, list) else []

    def _publish_angular(self, angular_z):
        twist = Twist()
        twist.angular.z = float(angular_z)
        self._cmd_pub.publish(twist)

    def _track_id_from_customer_id(self, customer_id):
        if not customer_id:
            return None
        match = _WAVE_CUSTOMER_ID_PATTERN.match(str(customer_id))
        if match is None:
            return None
        return int(match.group(1))

    def _center_u(self, track):
        center = track.get("smoothed_center", track.get("center_uv", [0.5, 0.5]))
        if not isinstance(center, list) or len(center) < 2:
            return 0.5
        return float(center[0])

    def _image_width(self, track):
        image_size = track.get("image_size", [640, 480])
        if isinstance(image_size, list) and len(image_size) >= 1:
            return int(image_size[0]) if int(image_size[0]) > 0 else 640
        return 640

    def _track_area(self, track):
        bbox = track.get("bbox", [0.0, 0.0, 0.0, 0.0])
        if not isinstance(bbox, list) or len(bbox) != 4:
            return 0.0
        return max(0.0, float(bbox[2]) - float(bbox[0])) * max(0.0, float(bbox[3]) - float(bbox[1]))

    def _allow_no_target_bypass(self):
        if self.allow_no_target_bypass:
            return True
        if not self.allow_no_target_bypass_in_mock_perception:
            return False
        return bool(rospy.get_param("/restaurant/perception/use_mock_perception", False))


_ALIGNMENT_HELPER = None


def get_alignment_helper():
    global _ALIGNMENT_HELPER
    if _ALIGNMENT_HELPER is None:
        _ALIGNMENT_HELPER = YawAlignmentHelper()
    return _ALIGNMENT_HELPER
