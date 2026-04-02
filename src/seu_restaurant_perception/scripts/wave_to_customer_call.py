#!/usr/bin/env python3
import copy
import json
import math
import threading

import numpy as np
import rospy
import tf2_geometry_msgs
import tf2_ros
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from seu_restaurant_msgs.msg import CustomerCall


class WaveToCustomerCallNode:
    def __init__(self):
        rospy.init_node("wave_to_customer_call")
        base_ns = "/restaurant/perception"
        self.wave_candidates_topic = rospy.get_param(base_ns + "/wave_candidates_topic", "/restaurant/perception/wave_candidates")
        self.wave_event_topic = rospy.get_param(base_ns + "/wave_event_topic", "/restaurant/perception/wave_event")
        self.customer_call_topic = rospy.get_param(base_ns + "/customer_call_topic", "/restaurant/customer_call")
        self.selection_debug_topic = rospy.get_param(base_ns + "/selection_debug_topic", "/restaurant/perception/selection_debug")
        self.target_uv_topic = rospy.get_param(base_ns + "/target_uv_topic", "/restaurant/perception/target_uv")
        self.person_point_topic = rospy.get_param(base_ns + "/person_point_topic", "/restaurant/perception/person_point")
        self.image_topic = rospy.get_param(base_ns + "/image_topic", "/camera/color/image_raw")

        self.position_frame = rospy.get_param(base_ns + "/position_frame", "camera_rgb_frame")
        self.camera_parent_frame = rospy.get_param(base_ns + "/camera_parent_frame", "base_link")
        self.camera_frame = rospy.get_param(base_ns + "/camera_frame", "camera_link")
        self.global_goal_frame = rospy.get_param(base_ns + "/global_goal_frame", "map")
        self.depth_topic = rospy.get_param(base_ns + "/depth_topic", "/camera/aligned_depth_to_color/image_raw")
        self.camera_info_topic = rospy.get_param(base_ns + "/camera_info_topic", "/camera/color/camera_info")
        self.use_depth_for_position = bool(rospy.get_param(base_ns + "/use_depth_for_position", True))
        self.allow_uv_fallback = bool(rospy.get_param(base_ns + "/allow_uv_fallback", True))
        self.stop_dist = float(rospy.get_param(base_ns + "/stop_dist", 0.75))
        self.depth_window_size = max(0, int(rospy.get_param(base_ns + "/depth_window_size", 2)))
        self.depth_min_m = float(rospy.get_param(base_ns + "/depth_min_m", 0.35))
        self.depth_max_m = float(rospy.get_param(base_ns + "/depth_max_m", 6.0))

        self.max_kept_customers = int(rospy.get_param(base_ns + "/max_kept_customers", 2))
        self.min_wave_score = float(rospy.get_param(base_ns + "/min_wave_score", 0.55))
        self.min_consecutive_frames = int(rospy.get_param(base_ns + "/min_consecutive_frames", 4))
        self.wave_event_threshold = float(
            rospy.get_param(base_ns + "/wave_detector/wave_event_threshold", rospy.get_param(base_ns + "/wave_event_threshold", 0.65))
        )
        self.publish_cooldown_sec = float(rospy.get_param(base_ns + "/publish_cooldown_sec", 3.0))
        self.top2_hold_bias = float(rospy.get_param(base_ns + "/top2_hold_bias", 0.10))
        self.top2_replace_margin = float(rospy.get_param(base_ns + "/top2_replace_margin", 0.12))
        self.min_active_weight = float(rospy.get_param(base_ns + "/min_active_weight", 0.15))
        self.table_regions = rospy.get_param(base_ns + "/table_regions", [])

        self.bridge = CvBridge()
        self._depth_lock = threading.Lock()
        self._latest_depth_image = None
        self._latest_depth_encoding = ""
        self._latest_depth_frame = ""
        self._latest_depth_stamp = rospy.Time(0)
        self._latest_camera_info = None
        self._latest_image_stamp = rospy.Time(0)
        self._latest_image_frame = ""
        self._last_goal_pose_by_track = {}
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.wave_event_pub = rospy.Publisher(self.wave_event_topic, CustomerCall, queue_size=10)
        self.customer_call_pub = rospy.Publisher(self.customer_call_topic, CustomerCall, queue_size=10)
        self.debug_pub = rospy.Publisher(self.selection_debug_topic, String, queue_size=10, latch=True)
        self.target_uv_pub = rospy.Publisher(self.target_uv_topic, PointStamped, queue_size=10)
        self.person_point_pub = rospy.Publisher(self.person_point_topic, PointStamped, queue_size=10)
        self.candidate_sub = rospy.Subscriber(self.wave_candidates_topic, String, self._candidate_callback, queue_size=10)
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self._image_callback, queue_size=1, buff_size=2 ** 24)
        self.depth_sub = rospy.Subscriber(self.depth_topic, Image, self._depth_callback, queue_size=1, buff_size=2 ** 24)
        self.camera_info_sub = rospy.Subscriber(self.camera_info_topic, CameraInfo, self._camera_info_callback, queue_size=5)

        self.last_published_track = {}
        self.held_track_ids = []
        rospy.loginfo(
            "[wave_to_customer_call] image=%s depth=%s camera_info=%s camera_parent=%s camera_frame=%s global_goal_frame=%s stop_dist=%.2f use_depth=%s uv_fallback=%s",
            self.image_topic,
            self.depth_topic,
            self.camera_info_topic,
            self.camera_parent_frame,
            self.camera_frame,
            self.global_goal_frame,
            self.stop_dist,
            self.use_depth_for_position,
            self.allow_uv_fallback,
        )
        rospy.loginfo(
            "[wave_to_customer_call] candidates=%s wave_event=%s customer_call=%s",
            self.wave_candidates_topic,
            self.wave_event_topic,
            self.customer_call_topic,
        )

    def spin(self):
        rospy.spin()

    def _image_callback(self, msg):
        self._latest_image_stamp = msg.header.stamp if msg.header.stamp.to_sec() > 0.0 else rospy.Time.now()
        self._latest_image_frame = msg.header.frame_id or self._latest_image_frame

    def _camera_info_callback(self, msg):
        with self._depth_lock:
            self._latest_camera_info = copy.deepcopy(msg)

    def _depth_callback(self, msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except CvBridgeError as exc:
            rospy.logwarn_throttle(2.0, "[wave_to_customer_call] depth cv_bridge failed: %s", exc)
            return
        with self._depth_lock:
            self._latest_depth_image = np.array(depth_image, copy=True)
            self._latest_depth_encoding = msg.encoding or ""
            self._latest_depth_frame = msg.header.frame_id or self.camera_frame
            self._latest_depth_stamp = msg.header.stamp if msg.header.stamp.to_sec() > 0.0 else rospy.Time.now()

    def _candidate_callback(self, msg):
        try:
            payload = json.loads(msg.data)
        except ValueError as exc:
            rospy.logwarn_throttle(2.0, "[wave_to_customer_call] invalid candidate json: %s", exc)
            return

        tracks = payload.get("tracks", [])
        if not isinstance(tracks, list):
            return

        stamp_sec = float(payload.get("header", {}).get("stamp", rospy.Time.now().to_sec()))
        frame_id = str(payload.get("header", {}).get("frame_id", self.position_frame))
        scored_tracks = [self._build_scored_track(track) for track in tracks]
        kept_tracks, selection_debug = self._select_top_tracks(scored_tracks)

        candidate_track_ids = [item["track_id"] for item in sorted(scored_tracks, key=lambda entry: entry["base_score"], reverse=True)]
        kept_track_ids = [item["track_id"] for item in kept_tracks]
        published_ids = []
        published_track_ids = []
        published_pose_modes = {}
        suppressed_track_ids = []
        suppressed_details = list(selection_debug["suppressed_reasons"])

        for item in kept_tracks:
            track = item["track"]
            track_id = item["track_id"]
            if not self._is_publishable(track, item["base_score"]):
                suppressed_track_ids.append(track_id)
                suppressed_details.append(
                    {
                        "track_id": track_id,
                        "reason": "not_publishable",
                        "state": str(track.get("state", "tentative")),
                        "base_score": round(item["base_score"], 4),
                        "smoothed_wave_score": round(float(track.get("smoothed_wave_score", 0.0)), 4),
                        "consecutive_wave_frames": int(track.get("consecutive_wave_frames", 0)),
                    }
                )
                continue
            if not self._cooldown_elapsed(track_id, stamp_sec):
                suppressed_track_ids.append(track_id)
                suppressed_details.append(
                    {
                        "track_id": track_id,
                        "reason": "cooldown",
                        "state": str(track.get("state", "tentative")),
                        "base_score": round(item["base_score"], 4),
                        "smoothed_wave_score": round(float(track.get("smoothed_wave_score", 0.0)), 4),
                        "consecutive_wave_frames": int(track.get("consecutive_wave_frames", 0)),
                    }
                )
                continue

            pose, pose_mode, person_point = self._resolve_track_pose(track, stamp_sec)
            if pose is None:
                suppressed_track_ids.append(track_id)
                suppressed_details.append(
                    {
                        "track_id": track_id,
                        "reason": "no_position",
                        "state": str(track.get("state", "tentative")),
                        "position_mode": pose_mode,
                    }
                )
                rospy.logwarn_throttle(
                    1.5,
                    "[wave_to_customer_call] track_id=%d has no valid position (mode=%s), skip publish",
                    track_id,
                    pose_mode,
                )
                continue

            customer_call = self._build_customer_call(track, frame_id, stamp_sec, item["base_score"], pose)
            self.wave_event_pub.publish(customer_call)
            self.customer_call_pub.publish(customer_call)
            self.last_published_track[track_id] = stamp_sec
            self._last_goal_pose_by_track[track_id] = copy.deepcopy(pose)
            published_ids.append(customer_call.customer_id)
            published_track_ids.append(track_id)
            published_pose_modes[str(track_id)] = pose_mode
            self._publish_track_debug_points(track, pose, person_point, frame_id, stamp_sec)
            rospy.loginfo(
                "[wave_to_customer_call] customer_id=%s track_id=%d final_goal frame=%s mode=%s x=%.3f y=%.3f yaw=%.3f",
                customer_call.customer_id,
                track_id,
                customer_call.position.header.frame_id,
                pose_mode,
                customer_call.position.pose.position.x,
                customer_call.position.pose.position.y,
                self._yaw_from_quat(customer_call.position.pose.orientation),
            )

        self.held_track_ids = kept_track_ids[: self.max_kept_customers]
        self.debug_pub.publish(
            String(
                data=json.dumps(
                    {
                        "header": payload.get("header", {}),
                        "candidate_track_ids": candidate_track_ids,
                        "candidate_track_states": {str(item["track_id"]): item["track"].get("state", "tentative") for item in scored_tracks},
                        "kept_track_ids": kept_track_ids,
                        "top2_ids": kept_track_ids[: self.max_kept_customers],
                        "published_customer_ids": published_ids,
                        "published_track_ids": published_track_ids,
                        "published_pose_modes": published_pose_modes,
                        "suppressed_track_ids": sorted(set(suppressed_track_ids)),
                        "top2_scores": [
                            {
                                "track_id": item["track_id"],
                                "state": item["track"].get("state", "tentative"),
                                "base_score": round(item["base_score"], 4),
                                "adjusted_score": round(item["adjusted_score"], 4),
                                "hold_bias_applied": bool(item["hold_bias_applied"]),
                                "event_confidence": round(float(item["track"].get("event_confidence", 0.0)), 4),
                                "wave_event": bool(item["track"].get("wave_event", False)),
                                "smoothed_wave_score": round(float(item["track"].get("smoothed_wave_score", 0.0)), 4),
                            }
                            for item in kept_tracks[: self.max_kept_customers]
                        ],
                        "kept_reasons": selection_debug["kept_reasons"],
                        "suppressed_reasons": suppressed_details,
                        "replacement_events": selection_debug["replacement_events"],
                        "tracks": [
                            {
                                "track_id": item["track_id"],
                                "state": item["track"].get("state", "tentative"),
                                "wave_score": round(float(item["track"].get("wave_score", 0.0)), 4),
                                "smoothed_wave_score": round(float(item["track"].get("smoothed_wave_score", 0.0)), 4),
                                "event_confidence": round(float(item["track"].get("event_confidence", 0.0)), 4),
                                "base_score": round(item["base_score"], 4),
                                "adjusted_score": round(item["adjusted_score"], 4),
                                "hold_bias_applied": bool(item["hold_bias_applied"]),
                                "consecutive_wave_frames": int(item["track"].get("consecutive_wave_frames", 0)),
                                "recent_visible": bool(item["track"].get("recent_visible", False)),
                                "is_waving": bool(item["track"].get("is_waving", False)),
                                "wave_event": bool(item["track"].get("wave_event", False)),
                                "published_customer_id": published_ids[published_track_ids.index(item["track_id"])]
                                if item["track_id"] in published_track_ids
                                else "",
                            }
                            for item in kept_tracks
                        ],
                    },
                    ensure_ascii=True,
                    sort_keys=True,
                )
            )
        )

    def _build_scored_track(self, track):
        track_id = int(track.get("track_id", 0))
        state = str(track.get("state", "tentative"))
        frames_seen = max(1.0, float(track.get("frames_seen", track.get("age", 1))))
        hits = float(track.get("hits", track.get("frames_seen", 0)))
        hit_ratio = min(1.0, hits / frames_seen)
        consecutive_frames = float(track.get("consecutive_wave_frames", 0))
        consecutive_score = min(1.0, consecutive_frames / max(1.0, float(self.min_consecutive_frames)))
        smoothed_wave_score = float(track.get("smoothed_wave_score", track.get("wave_score", 0.0)))
        event_confidence = float(track.get("event_confidence", smoothed_wave_score))
        event_bonus = 1.0 if bool(track.get("wave_event", False)) else 0.0
        recent_visible_score = 1.0 if bool(track.get("recent_visible", False) or track.get("visible", False)) else 0.0
        active_weight = 1.0 if state == "active" else (self.min_active_weight if state == "tentative" else 0.5 * self.min_active_weight)
        base_score = (
            0.34 * max(smoothed_wave_score, event_confidence)
            + 0.22 * hit_ratio
            + 0.16 * consecutive_score
            + 0.12 * recent_visible_score
            + 0.08 * active_weight
            + 0.08 * event_bonus
        )
        hold_bias_applied = track_id in self.held_track_ids and state in ("active", "lost") and recent_visible_score > 0.0
        adjusted_score = base_score + (self.top2_hold_bias if hold_bias_applied else 0.0)
        return {
            "track_id": track_id,
            "track": track,
            "state": state,
            "base_score": base_score,
            "adjusted_score": adjusted_score,
            "hit_ratio": hit_ratio,
            "hold_bias_applied": hold_bias_applied,
        }

    def _select_top_tracks(self, scored_tracks):
        kept = []
        kept_reasons = []
        suppressed_reasons = []
        replacement_events = []

        active_tracks = [item for item in scored_tracks if item["state"] == "active"]
        other_tracks = [item for item in scored_tracks if item["state"] != "active"]
        candidate_pool = sorted(active_tracks, key=lambda entry: entry["adjusted_score"], reverse=True)
        if len(candidate_pool) < self.max_kept_customers:
            candidate_pool.extend(sorted(other_tracks, key=lambda entry: entry["adjusted_score"], reverse=True))

        existing_map = {item["track_id"]: item for item in candidate_pool}
        for held_track_id in self.held_track_ids:
            item = existing_map.get(held_track_id)
            if item is None or len(kept) >= self.max_kept_customers:
                continue
            kept.append(item)
            kept_reasons.append(
                {
                    "track_id": item["track_id"],
                    "reason": "hold_existing_top2" if item["hold_bias_applied"] else "keep_existing_top2",
                    "state": item["state"],
                    "base_score": round(item["base_score"], 4),
                    "adjusted_score": round(item["adjusted_score"], 4),
                    "hold_bias_applied": bool(item["hold_bias_applied"]),
                }
            )

        for item in candidate_pool:
            if item["track_id"] in [entry["track_id"] for entry in kept]:
                continue
            if len(kept) < self.max_kept_customers:
                kept.append(item)
                kept_reasons.append(
                    {
                        "track_id": item["track_id"],
                        "reason": "fill_open_slot",
                        "state": item["state"],
                        "base_score": round(item["base_score"], 4),
                        "adjusted_score": round(item["adjusted_score"], 4),
                        "hold_bias_applied": bool(item["hold_bias_applied"]),
                    }
                )
                continue

            weakest = min(kept, key=lambda entry: entry["adjusted_score"])
            if item["adjusted_score"] > weakest["adjusted_score"] + self.top2_replace_margin:
                kept.remove(weakest)
                kept.append(item)
                replacement_events.append(
                    {
                        "replaced_track_id": weakest["track_id"],
                        "new_track_id": item["track_id"],
                        "old_adjusted_score": round(weakest["adjusted_score"], 4),
                        "new_adjusted_score": round(item["adjusted_score"], 4),
                        "margin": round(item["adjusted_score"] - weakest["adjusted_score"], 4),
                    }
                )
                kept_reasons.append(
                    {
                        "track_id": item["track_id"],
                        "reason": "replace_weaker_top2",
                        "state": item["state"],
                        "base_score": round(item["base_score"], 4),
                        "adjusted_score": round(item["adjusted_score"], 4),
                        "hold_bias_applied": bool(item["hold_bias_applied"]),
                    }
                )
                suppressed_reasons.append(
                    {
                        "track_id": weakest["track_id"],
                        "reason": "replaced_by_higher_score",
                        "state": weakest["state"],
                        "base_score": round(weakest["base_score"], 4),
                        "adjusted_score": round(weakest["adjusted_score"], 4),
                    }
                )
            else:
                suppressed_reasons.append(
                    {
                        "track_id": item["track_id"],
                        "reason": "hold_bias_blocked_replacement" if weakest["hold_bias_applied"] else "replace_margin_not_met",
                        "state": item["state"],
                        "base_score": round(item["base_score"], 4),
                        "adjusted_score": round(item["adjusted_score"], 4),
                        "compared_to_track_id": weakest["track_id"],
                        "required_score": round(weakest["adjusted_score"] + self.top2_replace_margin, 4),
                    }
                )

        kept.sort(key=lambda entry: entry["adjusted_score"], reverse=True)
        return kept[: self.max_kept_customers], {
            "kept_reasons": kept_reasons,
            "suppressed_reasons": suppressed_reasons,
            "replacement_events": replacement_events,
        }

    def _is_publishable(self, track, stability):
        event_confidence = float(track.get("event_confidence", track.get("smoothed_wave_score", track.get("wave_score", 0.0))))
        event_triggered = bool(track.get("wave_event", False))
        return bool(
            str(track.get("state", "tentative")) == "active"
            and (
                event_triggered
                or (
                    event_confidence >= self.wave_event_threshold
                    and float(track.get("smoothed_wave_score", track.get("wave_score", 0.0))) >= self.min_wave_score
                    and int(track.get("consecutive_wave_frames", 0)) >= self.min_consecutive_frames
                )
            )
            and stability >= min(self.min_wave_score, self.wave_event_threshold)
        )

    def _cooldown_elapsed(self, track_id, stamp_sec):
        last_stamp_sec = self.last_published_track.get(track_id, 0.0)
        return stamp_sec - last_stamp_sec >= self.publish_cooldown_sec

    def _resolve_track_pose(self, track, stamp_sec):
        track_id = int(track.get("track_id", -1))
        center_uv = track.get("smoothed_center", track.get("center_uv", [0.0, 0.0]))
        if len(center_uv) >= 2:
            rospy.loginfo_throttle(
                0.5,
                "[wave_to_customer_call] track_id=%d raw_uv=(%.4f, %.4f)",
                track_id,
                float(center_uv[0]),
                float(center_uv[1]),
            )
        if self.use_depth_for_position:
            depth_pose = self._build_depth_pose(track, stamp_sec)
            if depth_pose is not None:
                rospy.loginfo_throttle(
                    0.5,
                    "[wave_to_customer_call] track_id=%d person_3d frame=%s x=%.3f y=%.3f z=%.3f",
                    track_id,
                    depth_pose.header.frame_id,
                    depth_pose.pose.position.x,
                    depth_pose.pose.position.y,
                    depth_pose.pose.position.z,
                )
                goal_pose = self._build_goal_pose_from_person_pose(depth_pose, stamp_sec, track_id)
                if goal_pose is not None:
                    return goal_pose, "depth_3d_goal", depth_pose
        if self.allow_uv_fallback:
            if track_id in self._last_goal_pose_by_track:
                rospy.logwarn_throttle(
                    1.0,
                    "[wave_to_customer_call] track_id=%d fallback to last_valid_goal because depth/TF failed",
                    track_id,
                )
                return copy.deepcopy(self._last_goal_pose_by_track[track_id]), "last_valid_goal_fallback", None
            current_pose = self._build_current_pose_fallback(stamp_sec)
            if current_pose is not None:
                rospy.logwarn_throttle(
                    1.0,
                    "[wave_to_customer_call] track_id=%d fallback to current robot pose because depth/TF failed",
                    track_id,
                )
                return current_pose, "current_pose_fallback", None
            rospy.logwarn_throttle(
                1.0,
                "[wave_to_customer_call] track_id=%d fallback to uv pose because depth/TF and robot pose lookup failed",
                track_id,
            )
            return self._build_uv_pose(track, stamp_sec), "uv_fallback", None
        return None, "none", None

    def _build_uv_pose(self, track, stamp_sec):
        center_uv = track.get("smoothed_center", track.get("center_uv", [0.0, 0.0]))
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.from_sec(stamp_sec)
        pose.header.frame_id = self.camera_frame
        pose.pose.position.x = float(center_uv[0])
        pose.pose.position.y = float(center_uv[1])
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        return pose

    def _build_depth_pose(self, track, stamp_sec):
        with self._depth_lock:
            depth_image = None if self._latest_depth_image is None else np.array(self._latest_depth_image, copy=True)
            depth_encoding = self._latest_depth_encoding
            depth_frame = self._latest_depth_frame
            camera_info = copy.deepcopy(self._latest_camera_info)
        if depth_image is None or camera_info is None:
            return None
        if len(depth_image.shape) < 2:
            return None

        center_uv = track.get("smoothed_center", track.get("center_uv", [0.0, 0.0]))
        if len(center_uv) < 2:
            return None
        u_norm = max(0.0, min(1.0, float(center_uv[0])))
        v_norm = max(0.0, min(1.0, float(center_uv[1])))

        depth_h, depth_w = depth_image.shape[:2]
        if depth_w <= 1 or depth_h <= 1:
            return None
        u_px = int(round(u_norm * float(depth_w - 1)))
        v_px = int(round(v_norm * float(depth_h - 1)))

        depth_m = self._sample_depth_meters(depth_image, depth_encoding, u_px, v_px)
        if depth_m is None:
            return None

        fx = float(camera_info.K[0]) if len(camera_info.K) > 0 else 0.0
        fy = float(camera_info.K[4]) if len(camera_info.K) > 4 else 0.0
        cx = float(camera_info.K[2]) if len(camera_info.K) > 2 else 0.0
        cy = float(camera_info.K[5]) if len(camera_info.K) > 5 else 0.0
        if fx <= 1e-6 or fy <= 1e-6:
            rospy.logwarn_throttle(2.0, "[wave_to_customer_call] invalid camera intrinsics fx=%.6f fy=%.6f", fx, fy)
            return None

        info_w = int(camera_info.width) if int(camera_info.width) > 1 else depth_w
        info_h = int(camera_info.height) if int(camera_info.height) > 1 else depth_h
        u_info = u_norm * float(max(info_w - 1, 1))
        v_info = v_norm * float(max(info_h - 1, 1))
        x = (u_info - cx) * depth_m / fx
        y = (v_info - cy) * depth_m / fy
        z = depth_m

        pose = PoseStamped()
        pose.header.stamp = rospy.Time.from_sec(stamp_sec)
        pose.header.frame_id = camera_info.header.frame_id or depth_frame or self.camera_frame
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        pose.pose.orientation.w = 1.0
        return pose

    def _build_goal_pose_from_person_pose(self, person_pose_camera, stamp_sec, track_id):
        person_pose_base = self._transform_pose(person_pose_camera, self.camera_parent_frame, "person camera->base")
        if person_pose_base is None:
            return None
        person_pose_global = self._transform_pose(person_pose_base, self.global_goal_frame, "person base->global")
        if person_pose_global is None:
            return None

        px = float(person_pose_base.pose.position.x)
        py = float(person_pose_base.pose.position.y)
        dist = math.hypot(px, py)
        if dist <= 1e-3:
            rospy.logwarn_throttle(1.0, "[wave_to_customer_call] track_id=%d invalid person distance in base frame", track_id)
            return None

        approach_dist = max(0.0, dist - self.stop_dist)
        ux = px / dist
        uy = py / dist

        goal_pose_base = PoseStamped()
        goal_pose_base.header.stamp = rospy.Time.from_sec(stamp_sec)
        goal_pose_base.header.frame_id = self.camera_parent_frame
        goal_pose_base.pose.position.x = ux * approach_dist
        goal_pose_base.pose.position.y = uy * approach_dist
        goal_pose_base.pose.position.z = 0.0
        goal_pose_base.pose.orientation.w = 1.0

        goal_pose_global = self._transform_pose(goal_pose_base, self.global_goal_frame, "goal base->global")
        if goal_pose_global is None:
            return None

        dx = person_pose_global.pose.position.x - goal_pose_global.pose.position.x
        dy = person_pose_global.pose.position.y - goal_pose_global.pose.position.y
        yaw = math.atan2(dy, dx)
        goal_pose_global.pose.orientation = self._quat_from_yaw(yaw)
        goal_pose_global.header.stamp = rospy.Time.from_sec(stamp_sec)
        rospy.loginfo(
            "[wave_to_customer_call] track_id=%d stop_pose frame=%s x=%.3f y=%.3f yaw=%.3f",
            track_id,
            goal_pose_global.header.frame_id,
            goal_pose_global.pose.position.x,
            goal_pose_global.pose.position.y,
            yaw,
        )
        return goal_pose_global

    def _build_current_pose_fallback(self, stamp_sec):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_goal_frame,
                self.camera_parent_frame,
                rospy.Time(0),
                rospy.Duration(0.2),
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return None
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.from_sec(stamp_sec)
        pose.header.frame_id = self.global_goal_frame
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        return pose

    def _transform_pose(self, pose_msg, target_frame, tag):
        source_frame = pose_msg.header.frame_id or ""
        if not source_frame:
            return None
        if source_frame == target_frame:
            return copy.deepcopy(pose_msg)
        try:
            transformed = self.tf_buffer.transform(pose_msg, target_frame, rospy.Duration(0.2))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exc:
            pose_latest = copy.deepcopy(pose_msg)
            pose_latest.header.stamp = rospy.Time(0)
            try:
                transformed = self.tf_buffer.transform(pose_latest, target_frame, rospy.Duration(0.2))
                rospy.logwarn_throttle(
                    1.0,
                    "[wave_to_customer_call] %s TF fallback-used latest transform for %s -> %s",
                    tag,
                    source_frame,
                    target_frame,
                )
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logwarn_throttle(
                    1.0,
                    "[wave_to_customer_call] %s TF failed %s -> %s: %s",
                    tag,
                    source_frame,
                    target_frame,
                    exc,
                )
                return None
        transformed.header.stamp = pose_msg.header.stamp
        rospy.loginfo_throttle(
            0.5,
            "[wave_to_customer_call] %s TF converted %s -> %s",
            tag,
            source_frame,
            target_frame,
        )
        return transformed

    def _quat_from_yaw(self, yaw):
        quat = Quaternion()
        quat.x = 0.0
        quat.y = 0.0
        quat.z = math.sin(0.5 * yaw)
        quat.w = math.cos(0.5 * yaw)
        return quat

    def _yaw_from_quat(self, quat):
        siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _sample_depth_meters(self, depth_image, encoding, u_px, v_px):
        h, w = depth_image.shape[:2]
        x_min = max(0, u_px - self.depth_window_size)
        x_max = min(w - 1, u_px + self.depth_window_size)
        y_min = max(0, v_px - self.depth_window_size)
        y_max = min(h - 1, v_px + self.depth_window_size)
        window = np.array(depth_image[y_min : y_max + 1, x_min : x_max + 1], dtype=np.float32)
        if window.size == 0:
            return None

        enc = (encoding or "").upper()
        if "16U" in enc or enc == "MONO16":
            window = window * 0.001
        valid = window[np.isfinite(window)]
        valid = valid[valid > 1e-4]
        if self.depth_min_m > 0.0:
            valid = valid[valid >= self.depth_min_m]
        if self.depth_max_m > 0.0:
            valid = valid[valid <= self.depth_max_m]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    def _build_customer_call(self, track, frame_id, stamp_sec, stability_score, position_pose):
        track_id = int(track.get("track_id", 0))
        center_uv = track.get("smoothed_center", track.get("center_uv", [0.0, 0.0]))
        table_id = self._resolve_table_id(center_uv)

        msg = CustomerCall()
        msg.header.stamp = rospy.Time.from_sec(stamp_sec)
        msg.header.frame_id = frame_id
        msg.customer_id = "wave_track_{:02d}".format(track_id)
        msg.timestamp = msg.header.stamp
        msg.source = "wave"
        msg.table_id = table_id
        msg.position = position_pose
        msg.position.header.stamp = msg.header.stamp
        msg.confidence = min(
            1.0,
            max(
                float(track.get("event_confidence", 0.0)),
                float(track.get("smoothed_wave_score", track.get("wave_score", 0.0))),
                stability_score,
            ),
        )
        msg.requires_assistance = True
        return msg

    def _publish_track_debug_points(self, track, pose, person_point_pose, image_frame, stamp_sec):
        center_uv = track.get("smoothed_center", track.get("center_uv", [0.5, 0.5]))
        if len(center_uv) >= 2:
            uv_msg = PointStamped()
            uv_msg.header.stamp = rospy.Time.from_sec(stamp_sec)
            uv_msg.header.frame_id = image_frame
            uv_msg.point.x = float(center_uv[0])
            uv_msg.point.y = float(center_uv[1])
            uv_msg.point.z = 0.0
            self.target_uv_pub.publish(uv_msg)

        point_msg = PointStamped()
        if person_point_pose is not None:
            point_msg.header = person_point_pose.header
            point_msg.point.x = person_point_pose.pose.position.x
            point_msg.point.y = person_point_pose.pose.position.y
            point_msg.point.z = person_point_pose.pose.position.z
        else:
            point_msg.header = pose.header
            point_msg.point.x = pose.pose.position.x
            point_msg.point.y = pose.pose.position.y
            point_msg.point.z = pose.pose.position.z
        self.person_point_pub.publish(point_msg)

    def _resolve_table_id(self, center_uv):
        if len(center_uv) < 2:
            return ""
        u = float(center_uv[0])
        v = float(center_uv[1])
        for region in self.table_regions:
            u_min, u_max = region.get("u_range", [0.0, 1.0])
            v_min, v_max = region.get("v_range", [0.0, 1.0])
            if u_min <= u <= u_max and v_min <= v <= v_max:
                return str(region.get("table_id", ""))
        return ""


if __name__ == "__main__":
    WaveToCustomerCallNode().spin()
