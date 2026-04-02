#!/usr/bin/env python3
import json

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String

from seu_restaurant_msgs.msg import CustomerCall


def _safe_import_cv2():
    try:
        import cv2

        return cv2
    except ImportError:
        return None


def _customer_id_from_track(track_id):
    return "wave_track_{:02d}".format(int(track_id))


class PerceptionVisualizerNode:
    def __init__(self):
        rospy.init_node("perception_visualizer")
        base_ns = "/restaurant/perception"
        self.bridge = CvBridge()
        self.cv2 = _safe_import_cv2()
        self.image_topic = rospy.get_param(base_ns + "/image_topic", "/camera/rgb/image_raw")
        self.wave_candidates_topic = rospy.get_param(base_ns + "/wave_candidates_topic", "/restaurant/perception/wave_candidates")
        self.customer_call_topic = rospy.get_param(base_ns + "/customer_call_topic", "/restaurant/customer_call")
        self.customer_memory_debug_topic = rospy.get_param(base_ns + "/customer_memory_debug_topic", "/restaurant/customer_memory_debug")
        self.selection_debug_topic = rospy.get_param(base_ns + "/selection_debug_topic", "/restaurant/perception/selection_debug")
        self.debug_image_topic = rospy.get_param(base_ns + "/debug_image_topic", "/restaurant/perception/debug_image")
        self.max_kept_customers = int(rospy.get_param(base_ns + "/max_kept_customers", 2))

        self.show_skeleton = bool(rospy.get_param(base_ns + "/show_skeleton", True))
        self.show_bbox = bool(rospy.get_param(base_ns + "/show_bbox", True))
        self.show_wave_score = bool(rospy.get_param(base_ns + "/show_wave_score", rospy.get_param(base_ns + "/show_scores", True)))
        self.show_track_state = bool(rospy.get_param(base_ns + "/show_track_state", True))
        self.show_track_lifecycle = bool(rospy.get_param(base_ns + "/show_track_lifecycle", True))
        self.show_detector_source = bool(rospy.get_param(base_ns + "/show_detector_source", True))
        self.show_proposal_bbox = bool(rospy.get_param(base_ns + "/show_proposal_bbox", False))
        self.show_memory_panel = bool(rospy.get_param(base_ns + "/show_memory_panel", True))
        self.show_queue_panel = bool(rospy.get_param(base_ns + "/show_queue_panel", True))
        self.show_selected_customer = bool(rospy.get_param(base_ns + "/show_selected_customer", True))
        self.max_memory_rows = int(rospy.get_param(base_ns + "/max_memory_rows", 6))
        self.overlay_font_scale = float(rospy.get_param(base_ns + "/overlay_font_scale", 0.48))
        self.overlay_line_thickness = int(rospy.get_param(base_ns + "/overlay_line_thickness", 1))
        self.replacement_flash_sec = float(rospy.get_param(base_ns + "/replacement_flash_sec", 1.0))

        self.debug_pub = rospy.Publisher(self.debug_image_topic, Image, queue_size=1)
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self._image_callback, queue_size=1, buff_size=2 ** 24)
        self.candidate_sub = rospy.Subscriber(self.wave_candidates_topic, String, self._candidate_callback, queue_size=10)
        self.selection_sub = rospy.Subscriber(self.selection_debug_topic, String, self._selection_callback, queue_size=10)
        self.customer_call_sub = rospy.Subscriber(self.customer_call_topic, CustomerCall, self._customer_call_callback, queue_size=10)
        self.memory_sub = rospy.Subscriber(self.customer_memory_debug_topic, String, self._memory_callback, queue_size=10)

        self.latest_candidates = {}
        self.latest_selection = {}
        self.latest_customer_call = None
        self.latest_memory = {}
        self.latest_replacement_events = []

        rospy.loginfo("[perception_visualizer] debug_image_topic=%s", self.debug_image_topic)

    def spin(self):
        rospy.spin()

    def _candidate_callback(self, msg):
        try:
            self.latest_candidates = json.loads(msg.data)
        except ValueError:
            self.latest_candidates = {}

    def _selection_callback(self, msg):
        try:
            self.latest_selection = json.loads(msg.data)
        except ValueError:
            self.latest_selection = {}
        stamp_sec = float(self.latest_selection.get("header", {}).get("stamp", rospy.Time.now().to_sec()))
        for event in self.latest_selection.get("replacement_events", []):
            event_copy = dict(event)
            event_copy["stamp"] = stamp_sec
            self.latest_replacement_events.append(event_copy)
        self.latest_replacement_events = [
            event for event in self.latest_replacement_events
            if rospy.Time.now().to_sec() - float(event.get("stamp", 0.0)) <= self.replacement_flash_sec
        ]

    def _memory_callback(self, msg):
        try:
            self.latest_memory = json.loads(msg.data)
        except ValueError:
            self.latest_memory = {}

    def _customer_call_callback(self, msg):
        self.latest_customer_call = msg

    def _image_callback(self, msg):
        if self.cv2 is None:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            rospy.logwarn_throttle(2.0, "[perception_visualizer] cv_bridge failed: %s", exc)
            return

        tracks = self.latest_candidates.get("tracks", [])
        memory_entries = self.latest_memory.get("entries", [])
        image_h, image_w = frame.shape[:2]
        right_panel_w = 460 if self.show_memory_panel else 0
        bottom_panel_h = 160 if self.show_queue_panel else 0
        canvas = self.cv2.copyMakeBorder(frame, 0, bottom_panel_h, 0, right_panel_w, self.cv2.BORDER_CONSTANT, value=(18, 18, 18))

        top2_ids = set(self.latest_selection.get("top2_ids", self.latest_selection.get("kept_track_ids", [])[: self.max_kept_customers]))
        hold_ids = {
            int(item.get("track_id", 0))
            for item in self.latest_selection.get("top2_scores", [])
            if bool(item.get("hold_bias_applied", False))
        }
        published_track_ids = set(self.latest_selection.get("published_track_ids", []))
        published_customer_ids = set(self.latest_selection.get("published_customer_ids", []))
        raw_person_boxes = self.latest_candidates.get("raw_person_boxes", [])

        for box in raw_person_boxes:
            if not self.show_proposal_bbox:
                break
            if not isinstance(box, list) or len(box) != 4:
                continue
            x1 = int(float(box[0]) * image_w)
            y1 = int(float(box[1]) * image_h)
            x2 = int(float(box[2]) * image_w)
            y2 = int(float(box[3]) * image_h)
            self.cv2.rectangle(canvas, (x1, y1), (x2, y2), (110, 110, 110), 1)

        for track in tracks:
            track_id = int(track.get("track_id", 0))
            self._draw_track(
                canvas,
                track,
                image_w,
                image_h,
                is_top2=track_id in top2_ids,
                has_hold_bias=track_id in hold_ids,
                is_published=(track_id in published_track_ids or _customer_id_from_track(track_id) in published_customer_ids),
            )

        self._draw_status_panel(canvas, image_w, image_h, tracks, memory_entries)
        if self.show_memory_panel:
            self._draw_memory_panel(canvas, image_w, image_h, memory_entries, msg.header.stamp.to_sec())
        if self.show_queue_panel:
            self._draw_queue_panel(canvas, image_w, image_h)
        self._draw_replacement_events(canvas, image_w, image_h)

        try:
            debug_msg = self.bridge.cv2_to_imgmsg(canvas, encoding="bgr8")
            debug_msg.header = msg.header
            self.debug_pub.publish(debug_msg)
        except CvBridgeError as exc:
            rospy.logwarn_throttle(2.0, "[perception_visualizer] failed to publish debug image: %s", exc)

    def _draw_track(self, canvas, track, image_w, image_h, is_top2, has_hold_bias, is_published):
        bbox = track.get("bbox", [0.0, 0.0, 0.0, 0.0])
        proposal_bbox = track.get("proposal_bbox", bbox)
        center_uv = track.get("center_uv", [0.0, 0.0])
        keypoints = track.get("keypoints", {})
        is_waving = bool(track.get("is_waving", False))
        state = str(track.get("state", "tentative"))
        detector_mode = str(track.get("detector_mode", "-"))
        detector_backend = str(track.get("detector_backend", detector_mode))
        wave_event = bool(track.get("wave_event", False))
        color = (64, 100, 255)
        if state == "lost":
            color = (100, 100, 100)
        elif state == "tentative":
            color = (200, 120, 0)
        if is_waving:
            color = (0, 210, 90)
        if is_top2:
            color = (0, 215, 255)
        if is_published:
            color = (255, 190, 0)

        x1 = int(float(bbox[0]) * image_w)
        y1 = int(float(bbox[1]) * image_h)
        x2 = int(float(bbox[2]) * image_w)
        y2 = int(float(bbox[3]) * image_h)
        x1 = max(0, min(image_w - 1, x1))
        y1 = max(0, min(image_h - 1, y1))
        x2 = max(0, min(image_w - 1, x2))
        y2 = max(0, min(image_h - 1, y2))

        if self.show_bbox:
            self.cv2.rectangle(canvas, (x1, y1), (x2, y2), color, max(1, self.overlay_line_thickness + (1 if is_top2 else 0)))
        if self.show_proposal_bbox and proposal_bbox:
            px1 = int(float(proposal_bbox[0]) * image_w)
            py1 = int(float(proposal_bbox[1]) * image_h)
            px2 = int(float(proposal_bbox[2]) * image_w)
            py2 = int(float(proposal_bbox[3]) * image_h)
            self.cv2.rectangle(canvas, (px1, py1), (px2, py2), (140, 140, 140), 1)

        if self.show_skeleton:
            pairs = [
                ("left_shoulder", "right_shoulder"),
                ("left_shoulder", "left_elbow"),
                ("left_elbow", "left_wrist"),
                ("right_shoulder", "right_elbow"),
                ("right_elbow", "right_wrist"),
                ("left_shoulder", "left_hip"),
                ("right_shoulder", "right_hip"),
            ]
            for start_key, end_key in pairs:
                start = keypoints.get(start_key)
                end = keypoints.get(end_key)
                if start is None or end is None:
                    continue
                start_pt = (int(float(start[0]) * image_w), int(float(start[1]) * image_h))
                end_pt = (int(float(end[0]) * image_w), int(float(end[1]) * image_h))
                self.cv2.line(canvas, start_pt, end_pt, color, self.overlay_line_thickness + 1)
            for point in keypoints.values():
                self.cv2.circle(canvas, (int(float(point[0]) * image_w), int(float(point[1]) * image_h)), 3, color, -1)

        center_pt = (int(float(center_uv[0]) * image_w), int(float(center_uv[1]) * image_h))
        self.cv2.circle(canvas, center_pt, 4, color, -1)

        lines = ["track {} {}".format(int(track.get("track_id", 0)), "event" if wave_event else ("waving" if is_waving else "idle"))]
        if self.show_track_state:
            lines.append(
                "state={} phase={} top2={} hold={}".format(
                    state,
                    str(track.get("wave_phase", "-")),
                    "yes" if is_top2 else "no",
                    "yes" if has_hold_bias else "no",
                )
            )
        if self.show_track_lifecycle:
            lines.append(
                "hits={} miss={} cons_wave={} pub={}".format(
                    int(track.get("hits", 0)),
                    int(track.get("miss_count", 0)),
                    int(track.get("consecutive_wave_frames", 0)),
                    "yes" if is_published else "no",
                )
            )
        if self.show_wave_score:
            lines.insert(
                1,
                "wave={:.2f} smooth={:.2f} event={:.2f}".format(
                    float(track.get("wave_score", 0.0)),
                    float(track.get("smoothed_wave_score", 0.0)),
                    float(track.get("event_confidence", 0.0)),
                ),
            )
            lines.append(
                "flip={} amp={:.2f} cd={:.1f}s".format(
                    int(track.get("wave_flip_count", 0)),
                    float(track.get("wave_amplitude_norm", track.get("lateral_amplitude", 0.0))),
                    float(track.get("cooldown_remaining", 0.0)),
                )
            )
        if self.show_detector_source:
            lines.append(
                "src={} backend={} det={:.2f} prop={:.2f}".format(
                    detector_mode,
                    detector_backend,
                    float(track.get("detection_score", 0.0)),
                    float(track.get("proposal_score", 0.0)),
                )
            )
        if self.show_track_lifecycle:
            lines.append("age={} seen={:.1f}s".format(int(track.get("age", 0)), float(track.get("seconds_since_seen", 0.0))))

        label_x = x1
        label_y = max(24, y1 - 10)
        self._draw_text_block(canvas, lines, (label_x, label_y), color, anchor="bottom")

    def _draw_status_panel(self, canvas, image_w, image_h, tracks, memory_entries):
        waiting_ids = self.latest_memory.get("waiting_ids", [])
        selected_id = self.latest_memory.get("selected_id", "")
        serving_id = self.latest_memory.get("serving_id", "")
        served_ids = self.latest_memory.get("served_ids", [])
        counts = self.latest_candidates.get("track_counts", {})
        top2_ids = self.latest_selection.get("top2_ids", self.latest_selection.get("kept_track_ids", [])[: self.max_kept_customers])
        top2 = self.latest_selection.get("top2_scores", [])
        raw_person_boxes = self.latest_candidates.get("raw_person_boxes", [])
        top2_lines = []
        for item in top2[: self.max_kept_customers]:
            track_id = int(item.get("track_id", 0))
            top2_lines.append(
                "{} / t{} ({:.2f}{})".format(
                    _customer_id_from_track(track_id),
                    track_id,
                    float(item.get("adjusted_score", item.get("base_score", 0.0))),
                    " hold" if bool(item.get("hold_bias_applied", False)) else "",
                )
            )
        if not top2_lines:
            top2_lines.append("none")

        lines = [
            "Perception Debug Panel",
            "detector_backend={} detector_mode={}".format(
                self.latest_candidates.get("detector_backend", "-"),
                self.latest_candidates.get("detector_mode", "-"),
            ),
            "raw_boxes={} pose_success={} tracked={}".format(
                len(raw_person_boxes),
                int(self.latest_candidates.get("pose_success_count", 0)),
                int(self.latest_candidates.get("tracked_count", len(tracks))),
            ),
            "candidates={} active={} tentative={} lost={}".format(
                int(self.latest_candidates.get("candidates", len(tracks))),
                int(self.latest_candidates.get("active_count", counts.get("active", 0))),
                int(counts.get("tentative", 0)),
                int(counts.get("lost", 0)),
            ),
            "Top-2: {}".format("; ".join(top2_lines)),
            "top2_ids={}".format(top2_ids[: self.max_kept_customers]),
            "selected={}".format(selected_id or self.latest_memory.get("current_customer_id", "-") or "-"),
            "waiting={} serving={} served={}".format(len(waiting_ids), 1 if serving_id else 0, len(served_ids)),
        ]
        if self.show_selected_customer and self.latest_customer_call is not None:
            lines.append(
                "last_call={} conf={:.2f}".format(
                    self.latest_customer_call.customer_id or "-",
                    float(self.latest_customer_call.confidence),
                )
            )
        if serving_id:
            lines.append("serving={}".format(serving_id))
        if not memory_entries:
            lines.append("memory=unavailable")

        self._draw_panel(canvas, 8, 8, min(540, image_w - 16), 26 + 24 * len(lines), "System", lines)

    def _draw_memory_panel(self, canvas, image_w, image_h, memory_entries, stamp_sec):
        panel_x = image_w + 10
        panel_w = canvas.shape[1] - panel_x - 10
        header_lines = [
            "reason={}".format(self.latest_memory.get("reason", self.latest_selection.get("header", {}).get("frame_id", "-"))),
            "queue={}".format(self.latest_memory.get("queue_order", [])),
        ]
        if not memory_entries:
            header_lines.append("customer_memory_debug unavailable")
            self._draw_panel(canvas, panel_x, 8, panel_w, 92, "Memory", header_lines)
            return

        visible_entries = memory_entries[: max(1, self.max_memory_rows)]
        lines = list(header_lines)
        now_sec = stamp_sec if stamp_sec > 0.0 else rospy.Time.now().to_sec()
        for entry in visible_entries:
            queue_index = int(entry.get("queue_index", -1))
            queue_label = str(queue_index) if queue_index >= 0 else "-"
            lines.append(
                "{} {} conf={:.2f} q={} ev={}".format(
                    entry.get("customer_id", "?"),
                    entry.get("status", "?"),
                    float(entry.get("confidence", 0.0)),
                    queue_label,
                    int(entry.get("event_count", 0)),
                )
            )
            lines.append(
                "first={}s last={}s table={}".format(
                    self._relative_age(now_sec, entry.get("first_seen", 0.0)),
                    self._relative_age(now_sec, entry.get("last_seen", 0.0)),
                    entry.get("table_id", "-") or "-",
                )
            )

        panel_h = 34 + 22 * len(lines)
        self._draw_panel(canvas, panel_x, 8, panel_w, panel_h, "Memory", lines)

    def _draw_queue_panel(self, canvas, image_w, image_h):
        panel_y = image_h + 8
        panel_h = canvas.shape[0] - panel_y - 8
        queue_order = self.latest_memory.get("queue_order", [])
        kept_track_ids = self.latest_selection.get("kept_track_ids", [])
        suppressed_track_ids = self.latest_selection.get("suppressed_track_ids", [])
        published_customer_ids = self.latest_selection.get("published_customer_ids", [])
        selected_id = self.latest_memory.get("selected_id", "") or self.latest_memory.get("current_customer_id", "")
        kept_reasons = self.latest_selection.get("kept_reasons", [])
        replacement_events = self.latest_selection.get("replacement_events", [])

        lines = [
            "queue_order={}".format(queue_order if queue_order else []),
            "selected={}".format(selected_id or "-"),
            "kept_track_ids={}".format(kept_track_ids[: self.max_kept_customers]),
            "suppressed_track_ids={}".format(suppressed_track_ids),
            "published_customer_ids={}".format(published_customer_ids),
        ]
        if kept_reasons:
            lines.append("kept_reason={}".format(kept_reasons[0].get("reason", "-")))
        if replacement_events:
            latest = replacement_events[-1]
            lines.append(
                "replace t{}->t{} margin={:.2f}".format(
                    int(latest.get("replaced_track_id", 0)),
                    int(latest.get("new_track_id", 0)),
                    float(latest.get("margin", 0.0)),
                )
            )
        self._draw_panel(canvas, 8, panel_y, canvas.shape[1] - 16, panel_h, "Queue", lines)

    def _draw_replacement_events(self, canvas, image_w, image_h):
        active_events = [
            event for event in self.latest_replacement_events
            if rospy.Time.now().to_sec() - float(event.get("stamp", 0.0)) <= self.replacement_flash_sec
        ]
        if not active_events:
            return

        lines = [
            "Top-2 replacement",
        ]
        for event in active_events[-2:]:
            lines.append(
                "t{} -> t{} ({:.2f})".format(
                    int(event.get("replaced_track_id", 0)),
                    int(event.get("new_track_id", 0)),
                    float(event.get("margin", 0.0)),
                )
            )
        box_w = min(300, image_w - 16)
        box_h = 34 + 22 * len(lines)
        self._draw_panel(canvas, 8, max(60, image_h - box_h - 8), box_w, box_h, "Replacement", lines)

    def _draw_panel(self, canvas, x, y, w, h, title, lines):
        overlay = canvas.copy()
        self.cv2.rectangle(overlay, (x, y), (x + w, y + h), (32, 32, 32), -1)
        self.cv2.addWeighted(overlay, 0.55, canvas, 0.45, 0.0, canvas)
        self.cv2.rectangle(canvas, (x, y), (x + w, y + h), (180, 180, 180), 1)
        self.cv2.putText(canvas, title, (x + 10, y + 22), self.cv2.FONT_HERSHEY_SIMPLEX, self.overlay_font_scale + 0.06, (255, 255, 255), self.overlay_line_thickness + 1, self.cv2.LINE_AA)
        for index, line in enumerate(lines):
            baseline_y = y + 46 + index * 22
            self._draw_text(canvas, line, (x + 10, baseline_y), (240, 240, 240))

    def _draw_text_block(self, canvas, lines, origin, color, anchor="top"):
        x, y = origin
        line_height = 18
        block_height = line_height * len(lines)
        if anchor == "bottom":
            y = y - block_height
        background_top = max(0, y - 4)
        background_bottom = min(canvas.shape[0] - 1, y + block_height + 4)
        background_right = min(canvas.shape[1] - 1, x + 240)
        overlay = canvas.copy()
        self.cv2.rectangle(overlay, (max(0, x - 4), background_top), (background_right, background_bottom), (20, 20, 20), -1)
        self.cv2.addWeighted(overlay, 0.45, canvas, 0.55, 0.0, canvas)
        for index, line in enumerate(lines):
            self._draw_text(canvas, line, (x, y + 14 + index * line_height), color)

    def _draw_text(self, canvas, text, origin, color):
        self.cv2.putText(canvas, text, origin, self.cv2.FONT_HERSHEY_SIMPLEX, self.overlay_font_scale, (0, 0, 0), self.overlay_line_thickness + 2, self.cv2.LINE_AA)
        self.cv2.putText(canvas, text, origin, self.cv2.FONT_HERSHEY_SIMPLEX, self.overlay_font_scale, color, self.overlay_line_thickness, self.cv2.LINE_AA)

    def _relative_age(self, now_sec, seen_sec):
        if float(seen_sec) <= 0.0:
            return "-"
        return "{:.1f}".format(max(0.0, now_sec - float(seen_sec)))


if __name__ == "__main__":
    PerceptionVisualizerNode().spin()
