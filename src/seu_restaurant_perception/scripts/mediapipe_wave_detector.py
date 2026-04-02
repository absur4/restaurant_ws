#!/usr/bin/env python3
import json
import math
from collections import deque
from types import SimpleNamespace

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String


def _safe_import_cv2():
    try:
        import cv2

        return cv2
    except ImportError:
        return None


def _safe_import_mediapipe():
    try:
        import mediapipe as mp

        return mp
    except ImportError:
        return None


def _safe_import_ultralytics():
    try:
        from ultralytics import YOLO

        return YOLO
    except ImportError:
        return None


def _ema_vector(previous, current, alpha):
    if previous is None:
        return list(current)
    return [(1.0 - alpha) * float(previous[i]) + alpha * float(current[i]) for i in range(len(current))]


def _bbox_center(bbox):
    return [(float(bbox[0]) + float(bbox[2])) * 0.5, (float(bbox[1]) + float(bbox[3])) * 0.5]


def _bbox_size(bbox):
    return [max(1e-6, float(bbox[2]) - float(bbox[0])), max(1e-6, float(bbox[3]) - float(bbox[1]))]


def _bbox_area(bbox):
    width, height = _bbox_size(bbox)
    return width * height


def _bbox_iou(bbox_a, bbox_b):
    x1 = max(float(bbox_a[0]), float(bbox_b[0]))
    y1 = max(float(bbox_a[1]), float(bbox_b[1]))
    x2 = min(float(bbox_a[2]), float(bbox_b[2]))
    y2 = min(float(bbox_a[3]), float(bbox_b[3]))
    inter_w = max(0.0, x2 - x1)
    inter_h = max(0.0, y2 - y1)
    inter = inter_w * inter_h
    union = _bbox_area(bbox_a) + _bbox_area(bbox_b) - inter
    return inter / union if union > 1e-6 else 0.0


def _distance(point_a, point_b):
    return math.sqrt((float(point_a[0]) - float(point_b[0])) ** 2 + (float(point_a[1]) - float(point_b[1])) ** 2)


class TrackState:
    def __init__(self, track_id, detection, stamp_sec, history_size):
        self.track_id = track_id
        self.age = 1
        self.hits = 1
        self.frames_seen = 1
        self.miss_count = 0
        self.last_seen_stamp = stamp_sec
        self.last_center = list(detection["center_uv"])
        self.last_bbox = list(detection["bbox"])
        self.last_keypoint_center = list(detection["keypoint_center_uv"])
        self.smoothed_center = list(detection["center_uv"])
        self.smoothed_bbox = list(detection["bbox"])
        self.smoothed_wave_score = 0.0
        self.wave_score = 0.0
        self.event_confidence = 0.0
        self.is_waving = False
        self.wave_event = False
        self.hand_above_shoulder = False
        self.lateral_amplitude = 0.0
        self.state = "tentative"
        self.visible = True
        self.keypoints = detection["keypoints"]
        self.detector_mode = str(detection.get("detector_mode", "single_pose_fallback"))
        self.detector_backend = str(detection.get("detector_backend", self.detector_mode))
        self.proposal_bbox = list(detection.get("proposal_bbox", detection["bbox"]))
        self.crop_bbox = list(detection.get("crop_bbox", detection["bbox"]))
        self.detection_score = float(detection.get("detection_score", 1.0))
        self.proposal_score = float(detection.get("proposal_score", self.detection_score))
        self.wrist_history = deque(maxlen=history_size)
        self.wrist_x_hist = deque(maxlen=history_size)
        self.wrist_y_hist = deque(maxlen=history_size)
        self.shoulder_x_hist = deque(maxlen=history_size)
        self.shoulder_y_hist = deque(maxlen=history_size)
        self.elbow_x_hist = deque(maxlen=history_size)
        self.elbow_y_hist = deque(maxlen=history_size)
        self.timestamp_hist = deque(maxlen=history_size)
        self.raised_hist = deque(maxlen=history_size)
        self.x_norm_hist = deque(maxlen=history_size)
        self.hand_raised_frames = 0
        self.wave_flip_count = 0
        self.wave_amplitude_norm = 0.0
        self.consecutive_wave_frames = 0
        self.recent_visible_streak = 1
        self.last_match_cost = 0.0
        self.last_wave_event_time = 0.0
        self.cooldown_until = 0.0
        self.wave_phase = "idle"
        self.wave_side = ""

    @property
    def bbox(self):
        return self.last_bbox

    @property
    def center_uv(self):
        return self.last_center


class MediaPipeWaveDetectorNode:
    def __init__(self):
        rospy.init_node("mediapipe_wave_detector")
        self.bridge = CvBridge()
        self.cv2 = _safe_import_cv2()
        self.mp = _safe_import_mediapipe()
        self.YOLO = _safe_import_ultralytics()

        base_ns = "/restaurant/perception"
        detector_ns = base_ns + "/wave_detector"
        self.image_topic = rospy.get_param(base_ns + "/image_topic", "/camera/rgb/image_raw")
        self.wave_candidates_topic = rospy.get_param(base_ns + "/wave_candidates_topic", "/restaurant/perception/wave_candidates")
        self.publish_rate = float(rospy.get_param(base_ns + "/publish_rate", 10.0))
        self.use_mediapipe_hands = bool(rospy.get_param(base_ns + "/use_mediapipe_hands", False))
        self.min_wave_score = float(rospy.get_param(base_ns + "/min_wave_score", 0.55))
        self.min_consecutive_frames = int(rospy.get_param(base_ns + "/min_consecutive_frames", 4))
        self.track_timeout_sec = float(rospy.get_param(base_ns + "/track_timeout_sec", 0.8))
        self.track_match_distance_norm = float(rospy.get_param(detector_ns + "/track_match_distance_norm", 0.12))
        self.track_match_distance_thresh = float(rospy.get_param(base_ns + "/track_match_distance_thresh", self.track_match_distance_norm))
        self.track_iou_thresh = float(rospy.get_param(base_ns + "/track_iou_thresh", 0.08))
        self.track_max_miss_count = int(rospy.get_param(base_ns + "/track_max_miss_count", 4))
        self.track_activate_hits = int(rospy.get_param(base_ns + "/track_activate_hits", 3))
        self.track_lost_timeout_sec = float(rospy.get_param(base_ns + "/track_lost_timeout_sec", 1.2))
        self.center_ema_alpha = float(rospy.get_param(base_ns + "/center_ema_alpha", 0.32))
        self.bbox_ema_alpha = float(rospy.get_param(base_ns + "/bbox_ema_alpha", self.center_ema_alpha))
        self.score_ema_alpha = float(rospy.get_param(base_ns + "/score_ema_alpha", 0.28))
        self.hand_above_shoulder_thresh = float(rospy.get_param(base_ns + "/hand_above_shoulder_thresh", 0.04))
        self.lateral_wave_amplitude_thresh = float(rospy.get_param(base_ns + "/lateral_wave_amplitude_thresh", 0.08))
        self.history_size = int(rospy.get_param(detector_ns + "/history_size", 12))
        self.max_num_poses = int(rospy.get_param(detector_ns + "/max_num_poses", 4))
        self.detector_mode = str(rospy.get_param(detector_ns + "/detector_mode", "person_crop_pose"))
        self.detector_backend = str(rospy.get_param(detector_ns + "/detector_backend", "yolo26_mediapipe"))
        self.enable_detector_fallback = bool(rospy.get_param(detector_ns + "/enable_detector_fallback", True))
        self.min_pose_detection_confidence = float(rospy.get_param(detector_ns + "/min_pose_detection_confidence", 0.5))
        self.min_pose_presence_confidence = float(rospy.get_param(detector_ns + "/min_pose_presence_confidence", 0.5))
        self.min_tracking_confidence = float(rospy.get_param(detector_ns + "/min_tracking_confidence", 0.5))
        self.person_detector_stride = float(rospy.get_param(detector_ns + "/person_detector_stride", 8.0))
        self.person_detector_padding = float(rospy.get_param(detector_ns + "/person_detector_padding", 16.0))
        self.person_detector_scale = float(rospy.get_param(detector_ns + "/person_detector_scale", 1.05))
        self.person_detector_hit_threshold = float(rospy.get_param(detector_ns + "/person_detector_hit_threshold", 0.0))
        self.person_detector_score_thresh = float(rospy.get_param(detector_ns + "/person_detector_score_thresh", 0.0))
        self.person_detector_nms_thresh = float(rospy.get_param(detector_ns + "/person_detector_nms_thresh", 0.45))
        self.max_person_detections = int(rospy.get_param(detector_ns + "/max_person_detections", 4))
        self.crop_expand_ratio = float(rospy.get_param(detector_ns + "/crop_expand_ratio", 0.18))
        self.crop_min_size_px = int(rospy.get_param(detector_ns + "/crop_min_size_px", 96))
        self.crop_pose_min_visibility = float(rospy.get_param(detector_ns + "/crop_pose_min_visibility", 0.2))
        self.pose_reject_min_shoulder_width = float(rospy.get_param(detector_ns + "/pose_reject_min_shoulder_width", 0.03))
        self.pose_duplicate_iou_thresh = float(rospy.get_param(detector_ns + "/pose_duplicate_iou_thresh", 0.65))
        self.pose_duplicate_center_thresh = float(rospy.get_param(detector_ns + "/pose_duplicate_center_thresh", 0.06))
        self.yolo_model = str(rospy.get_param(detector_ns + "/yolo_model", "yolo26n.pt"))
        self.yolo_conf = float(rospy.get_param(detector_ns + "/yolo_conf", 0.35))
        self.yolo_imgsz = int(rospy.get_param(detector_ns + "/yolo_imgsz", 640))
        self.yolo_device = str(rospy.get_param(detector_ns + "/yolo_device", "cpu"))
        self.wave_window_sec = float(rospy.get_param(detector_ns + "/wave_window_sec", 1.0))
        self.min_raise_frames = int(rospy.get_param(detector_ns + "/min_raise_frames", 4))
        self.min_wave_flips = int(rospy.get_param(detector_ns + "/min_wave_flips", 2))
        self.min_wave_amplitude_norm = float(rospy.get_param(detector_ns + "/min_wave_amplitude_norm", 0.18))
        self.wave_event_threshold = float(rospy.get_param(detector_ns + "/wave_event_threshold", 0.65))
        self.wave_cooldown_sec = float(rospy.get_param(detector_ns + "/wave_cooldown_sec", 1.2))

        self.candidate_pub = rospy.Publisher(self.wave_candidates_topic, String, queue_size=10)
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self._image_callback, queue_size=1, buff_size=2 ** 24)

        self.tracks = {}
        self.next_track_id = 1
        self.last_publish_sec = 0.0
        self.pose_landmarker = None
        self.pose_detector = None
        self.crop_pose_detector = None
        self.hands_detector = None
        self.person_detector = None
        self.yolo_detector = None
        self.last_raw_person_boxes = []
        self.last_pose_success_count = 0
        self.last_detector_mode_used = self.detector_mode
        self.last_detector_backend_used = self.detector_backend
        self._init_mediapipe()
        self._init_person_detector()

        rospy.loginfo(
            "[mediapipe_wave_detector] image_topic=%s wave_candidates_topic=%s mediapipe_available=%s detector_backend=%s yolo_ready=%s",
            self.image_topic,
            self.wave_candidates_topic,
            self.mp is not None,
            self.detector_backend,
            self.yolo_detector is not None,
        )

    def _init_mediapipe(self):
        if self.mp is None:
            rospy.logwarn("[mediapipe_wave_detector] mediapipe is not installed; detector will publish empty candidates")
            return

        self.pose_detector = None
        self.crop_pose_detector = None
        self.hands_detector = None
        try:
            tasks = self.mp.tasks
            vision = tasks.vision
            base_options = tasks.BaseOptions(model_asset_path=rospy.get_param("/restaurant/perception/wave_detector/pose_landmarker_model_path", ""))
            if base_options.model_asset_path:
                options = vision.PoseLandmarkerOptions(
                    base_options=base_options,
                    running_mode=vision.RunningMode.IMAGE,
                    num_poses=self.max_num_poses,
                    min_pose_detection_confidence=self.min_pose_detection_confidence,
                    min_pose_presence_confidence=self.min_pose_presence_confidence,
                    min_tracking_confidence=self.min_tracking_confidence,
                    output_segmentation_masks=False,
                )
                self.pose_landmarker = vision.PoseLandmarker.create_from_options(options)
        except Exception as exc:
            rospy.logwarn("[mediapipe_wave_detector] PoseLandmarker unavailable: %s", exc)
            self.pose_landmarker = None

        self.pose_detector = self.mp.solutions.pose.Pose(
            static_image_mode=False,
            model_complexity=1,
            enable_segmentation=False,
            min_detection_confidence=self.min_pose_detection_confidence,
            min_tracking_confidence=self.min_tracking_confidence,
        )
        self.crop_pose_detector = self.mp.solutions.pose.Pose(
            static_image_mode=True,
            model_complexity=1,
            enable_segmentation=False,
            min_detection_confidence=self.min_pose_detection_confidence,
            min_tracking_confidence=self.min_tracking_confidence,
        )

        if self.use_mediapipe_hands:
            self.hands_detector = self.mp.solutions.hands.Hands(
                static_image_mode=False,
                max_num_hands=4,
                min_detection_confidence=self.min_pose_detection_confidence,
                min_tracking_confidence=self.min_tracking_confidence,
            )

    def _init_person_detector(self):
        if self.cv2 is not None and hasattr(self.cv2, "HOGDescriptor"):
            try:
                self.person_detector = self.cv2.HOGDescriptor()
                self.person_detector.setSVMDetector(self.cv2.HOGDescriptor_getDefaultPeopleDetector())
            except Exception as exc:
                rospy.logwarn("[mediapipe_wave_detector] HOG person detector unavailable: %s", exc)
                self.person_detector = None

        if self.YOLO is None:
            rospy.logwarn("[mediapipe_wave_detector] ultralytics is not installed; YOLO26 backend disabled")
            return

        try:
            self.yolo_detector = self.YOLO(self.yolo_model)
        except Exception as exc:
            self.yolo_detector = None
            rospy.logwarn(
                "[mediapipe_wave_detector] YOLO26 init failed for model=%s, fallback will be used: %s",
                self.yolo_model,
                exc,
            )

    def spin(self):
        rospy.spin()

    def _image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            rospy.logwarn_throttle(2.0, "[mediapipe_wave_detector] cv_bridge failed: %s", exc)
            return

        stamp_sec = msg.header.stamp.to_sec() if msg.header.stamp.to_sec() > 0.0 else rospy.Time.now().to_sec()
        detections = self._detect_people(frame)
        tracks = self._update_tracks(detections, stamp_sec, frame.shape[1], frame.shape[0])
        self._publish_candidates(msg, tracks, stamp_sec)

    def _detect_people(self, frame):
        if self.mp is None or self.cv2 is None:
            return []

        rgb = self.cv2.cvtColor(frame, self.cv2.COLOR_BGR2RGB)
        image_h, image_w = frame.shape[:2]
        self.last_raw_person_boxes = []
        self.last_pose_success_count = 0

        plan = [self.detector_backend]
        if self.enable_detector_fallback:
            if self.detector_backend == "yolo26_mediapipe":
                plan.extend(["hog_mediapipe", "single_pose_fallback", "landmarker_multi"])
            elif self.detector_backend == "hog_mediapipe":
                plan.extend(["single_pose_fallback", "landmarker_multi"])
            else:
                plan.extend(["yolo26_mediapipe", "hog_mediapipe", "single_pose_fallback", "landmarker_multi"])

        detections = []
        for backend in plan:
            if backend == "yolo26_mediapipe":
                detections = self._detect_people_from_person_boxes(frame, rgb, image_w, image_h, backend)
            elif backend == "hog_mediapipe":
                detections = self._detect_people_from_person_boxes(frame, rgb, image_w, image_h, backend)
            elif backend == "landmarker_multi":
                detections = self._detect_people_landmarker_multi(rgb, image_w, image_h)
            else:
                detections = self._detect_people_single_pose(rgb, image_w, image_h)
            if detections:
                self.last_detector_backend_used = backend
                self.last_detector_mode_used = detections[0].get("detector_mode", backend)
                return self._deduplicate_detections(detections)

        self.last_detector_backend_used = plan[-1]
        self.last_detector_mode_used = plan[-1]
        return []

    def _detect_people_from_person_boxes(self, frame, rgb, image_w, image_h, backend):
        if backend == "yolo26_mediapipe":
            boxes = self._detect_person_boxes_yolo26(rgb)
        else:
            boxes = self._detect_person_boxes_hog(frame)
        self.last_raw_person_boxes = [box["bbox"] for box in boxes]

        detections = []
        pose_success_count = 0
        for idx, box in enumerate(boxes):
            landmarks = self._run_mediapipe_on_crop(rgb, box)
            if landmarks is None:
                continue
            full_landmarks = self._landmarks_crop_to_full_image(landmarks, box["crop_bbox_px"], image_w, image_h)
            detection = self._build_detection_from_landmarks(
                full_landmarks,
                image_w,
                image_h,
                detector_id=idx,
                detector_mode="person_crop_pose",
                detector_backend=backend,
                proposal_bbox=box["bbox"],
                crop_bbox=box["crop_bbox"],
                detection_score=box["score"],
                proposal_score=box["score"],
            )
            if detection and self._validate_detection(detection):
                detections.append(detection)
                pose_success_count += 1
        self.last_pose_success_count = pose_success_count
        return detections

    def _detect_people_landmarker_multi(self, rgb, image_w, image_h):
        detections = []
        if self.pose_landmarker is None:
            return detections
        try:
            mp_image = self.mp.Image(image_format=self.mp.ImageFormat.SRGB, data=rgb)
            result = self.pose_landmarker.detect(mp_image)
            for idx, landmarks in enumerate(result.pose_landmarks):
                detection = self._build_detection_from_landmarks(
                    landmarks,
                    image_w,
                    image_h,
                    detector_id=idx,
                    detector_mode="landmarker_multi",
                    detector_backend="landmarker_multi",
                    proposal_bbox=None,
                    crop_bbox=None,
                    detection_score=1.0,
                    proposal_score=1.0,
                )
                if detection and self._validate_detection(detection):
                    detections.append(detection)
            self.last_pose_success_count = len(detections)
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "[mediapipe_wave_detector] PoseLandmarker detect failed: %s", exc)
        return detections

    def _detect_people_single_pose(self, rgb, image_w, image_h):
        detections = []
        if self.pose_detector is None:
            return detections
        result = self.pose_detector.process(rgb)
        if result.pose_landmarks:
            detection = self._build_detection_from_landmarks(
                result.pose_landmarks.landmark,
                image_w,
                image_h,
                detector_id=0,
                detector_mode="single_pose_fallback",
                detector_backend="single_pose_fallback",
                proposal_bbox=None,
                crop_bbox=None,
                detection_score=1.0,
                proposal_score=1.0,
            )
            if detection and self._validate_detection(detection):
                detections.append(detection)
        self.last_pose_success_count = len(detections)
        return detections

    def _detect_person_boxes_yolo26(self, rgb):
        if self.yolo_detector is None:
            return []
        try:
            results = self.yolo_detector.predict(
                source=rgb,
                conf=max(0.01, float(self.yolo_conf)),
                imgsz=max(64, int(self.yolo_imgsz)),
                device=self.yolo_device,
                classes=[0],
                verbose=False,
            )
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "[mediapipe_wave_detector] YOLO26 detect failed, fallback will be used: %s", exc)
            return []

        if not results:
            return []

        image_h, image_w = rgb.shape[:2]
        proposals = []
        result = results[0]
        boxes = getattr(result, "boxes", None)
        if boxes is None:
            return []
        xyxy_values = boxes.xyxy.tolist() if hasattr(boxes.xyxy, "tolist") else list(boxes.xyxy)
        conf_values = boxes.conf.tolist() if hasattr(boxes.conf, "tolist") else list(boxes.conf)
        cls_values = boxes.cls.tolist() if hasattr(boxes.cls, "tolist") else list(boxes.cls)
        for idx, xyxy in enumerate(xyxy_values):
            cls_id = int(cls_values[idx]) if idx < len(cls_values) else 0
            if cls_id != 0:
                continue
            x1, y1, x2, y2 = [float(value) for value in xyxy]
            score = float(conf_values[idx]) if idx < len(conf_values) else 1.0
            x1 = max(0, min(image_w - 1, int(round(x1))))
            y1 = max(0, min(image_h - 1, int(round(y1))))
            x2 = max(x1 + 1, min(image_w, int(round(x2))))
            y2 = max(y1 + 1, min(image_h, int(round(y2))))
            proposals.append(
                {
                    "bbox_px": [x1, y1, x2, y2],
                    "bbox": [x1 / float(image_w), y1 / float(image_h), x2 / float(image_w), y2 / float(image_h)],
                    "score": score,
                }
            )

        proposals.sort(key=lambda item: item["score"], reverse=True)
        proposals = self._nms_boxes(proposals, self.person_detector_nms_thresh)
        enriched = []
        for proposal in proposals[: max(1, self.max_person_detections)]:
            crop_bbox_px = self._expand_and_clip_bbox_px(proposal["bbox_px"], image_w, image_h, self.crop_expand_ratio)
            enriched.append(
                {
                    "bbox_px": proposal["bbox_px"],
                    "bbox": proposal["bbox"],
                    "score": proposal["score"],
                    "crop_bbox_px": crop_bbox_px,
                    "crop_bbox": [
                        crop_bbox_px[0] / float(image_w),
                        crop_bbox_px[1] / float(image_h),
                        crop_bbox_px[2] / float(image_w),
                        crop_bbox_px[3] / float(image_h),
                    ],
                }
            )
        return enriched

    def _detect_person_boxes_hog(self, frame):
        if self.person_detector is None or self.cv2 is None:
            return []
        stride = (max(4, int(self.person_detector_stride)), max(4, int(self.person_detector_stride)))
        padding = (max(0, int(self.person_detector_padding)), max(0, int(self.person_detector_padding)))
        try:
            rects, weights = self.person_detector.detectMultiScale(
                frame,
                winStride=stride,
                padding=padding,
                scale=max(1.01, float(self.person_detector_scale)),
                hitThreshold=float(self.person_detector_hit_threshold),
            )
        except TypeError:
            rects, weights = self.person_detector.detectMultiScale(
                frame,
                winStride=stride,
                padding=padding,
                scale=max(1.01, float(self.person_detector_scale)),
            )

        image_h, image_w = frame.shape[:2]
        proposals = []
        for idx, rect in enumerate(rects):
            x, y, w, h = [int(value) for value in rect]
            score = float(weights[idx]) if idx < len(weights) else 1.0
            if score < self.person_detector_score_thresh:
                continue
            x1 = max(0, min(image_w - 1, x))
            y1 = max(0, min(image_h - 1, y))
            x2 = max(x1 + 1, min(image_w, x + w))
            y2 = max(y1 + 1, min(image_h, y + h))
            crop_bbox_px = self._expand_and_clip_bbox_px([x1, y1, x2, y2], image_w, image_h, self.crop_expand_ratio)
            proposals.append(
                {
                    "bbox_px": [x1, y1, x2, y2],
                    "bbox": [x1 / float(image_w), y1 / float(image_h), x2 / float(image_w), y2 / float(image_h)],
                    "score": score,
                    "crop_bbox_px": crop_bbox_px,
                    "crop_bbox": [
                        crop_bbox_px[0] / float(image_w),
                        crop_bbox_px[1] / float(image_h),
                        crop_bbox_px[2] / float(image_w),
                        crop_bbox_px[3] / float(image_h),
                    ],
                }
            )
        proposals.sort(key=lambda item: item["score"], reverse=True)
        proposals = self._nms_boxes(proposals, self.person_detector_nms_thresh)
        return proposals[: max(1, self.max_person_detections)]

    def _nms_boxes(self, proposals, iou_thresh):
        kept = []
        for proposal in proposals:
            if any(_bbox_iou(existing["bbox"], proposal["bbox"]) >= iou_thresh for existing in kept):
                continue
            kept.append(proposal)
        return kept

    def _expand_and_clip_bbox_px(self, bbox_px, image_w, image_h, expand_ratio):
        x1, y1, x2, y2 = [int(value) for value in bbox_px]
        width = max(1, x2 - x1)
        height = max(1, y2 - y1)
        expand_w = int(round(width * expand_ratio))
        expand_h = int(round(height * expand_ratio))
        return [
            max(0, x1 - expand_w),
            max(0, y1 - expand_h),
            min(image_w, x2 + expand_w),
            min(image_h, y2 + expand_h),
        ]

    def _run_mediapipe_on_crop(self, rgb, box):
        if self.crop_pose_detector is None:
            return None
        crop_bbox_px = box.get("crop_bbox_px", box.get("bbox_px"))
        if crop_bbox_px is None:
            return None
        x1, y1, x2, y2 = crop_bbox_px
        crop_w = x2 - x1
        crop_h = y2 - y1
        if crop_w < self.crop_min_size_px or crop_h < self.crop_min_size_px:
            return None
        crop_rgb = rgb[y1:y2, x1:x2]
        if crop_rgb is None or crop_rgb.size == 0:
            return None
        result = self.crop_pose_detector.process(crop_rgb)
        if not result.pose_landmarks:
            return None
        return result.pose_landmarks.landmark

    def _landmarks_crop_to_full_image(self, landmarks, crop_bbox_px, image_w, image_h):
        x1, y1, x2, y2 = crop_bbox_px
        crop_w = max(1.0, float(x2 - x1))
        crop_h = max(1.0, float(y2 - y1))
        remapped = []
        for landmark in landmarks:
            landmark_copy = SimpleNamespace()
            landmark_copy.x = (float(landmark.x) * crop_w + float(x1)) / max(float(image_w), 1.0)
            landmark_copy.y = (float(landmark.y) * crop_h + float(y1)) / max(float(image_h), 1.0)
            landmark_copy.z = float(getattr(landmark, "z", 0.0))
            landmark_copy.visibility = float(getattr(landmark, "visibility", 1.0))
            landmark_copy.presence = float(getattr(landmark, "presence", 1.0))
            remapped.append(landmark_copy)
        return remapped

    def _deduplicate_detections(self, detections):
        deduped = []
        for detection in sorted(detections, key=lambda item: float(item.get("detection_score", 0.0)), reverse=True):
            duplicate = False
            for kept in deduped:
                if _bbox_iou(kept["bbox"], detection["bbox"]) >= self.pose_duplicate_iou_thresh:
                    duplicate = True
                    break
                if _distance(kept["center_uv"], detection["center_uv"]) <= self.pose_duplicate_center_thresh:
                    duplicate = True
                    break
            if not duplicate:
                deduped.append(detection)
        return deduped[: max(1, self.max_num_poses, self.max_person_detections)]

    def _validate_detection(self, detection):
        keypoints = detection.get("keypoints", {})
        left_shoulder = keypoints.get("left_shoulder")
        right_shoulder = keypoints.get("right_shoulder")
        if left_shoulder is None or right_shoulder is None:
            return False
        visible_keypoints = [point for point in keypoints.values() if float(point[2]) >= self.crop_pose_min_visibility]
        if len(visible_keypoints) < 4:
            return False
        shoulder_width = abs(float(left_shoulder[0]) - float(right_shoulder[0]))
        if shoulder_width < self.pose_reject_min_shoulder_width:
            return False
        bbox = detection.get("bbox", [0.0, 0.0, 0.0, 0.0])
        if _bbox_area(bbox) <= 1e-4:
            return False
        return True

    def _build_detection_from_landmarks(
        self,
        landmarks,
        image_w,
        image_h,
        detector_id,
        detector_mode,
        detector_backend,
        proposal_bbox,
        crop_bbox,
        detection_score,
        proposal_score,
    ):
        keypoint_ids = {
            "nose": 0,
            "left_shoulder": 11,
            "right_shoulder": 12,
            "left_elbow": 13,
            "right_elbow": 14,
            "left_wrist": 15,
            "right_wrist": 16,
            "left_hip": 23,
            "right_hip": 24,
        }
        keypoints = {}
        uv_points = []
        weighted_center = [0.0, 0.0]
        visibility_sum = 0.0
        for name, index in keypoint_ids.items():
            if index >= len(landmarks):
                continue
            landmark = landmarks[index]
            visibility = float(getattr(landmark, "visibility", 1.0))
            u = float(max(0.0, min(1.0, landmark.x)))
            v = float(max(0.0, min(1.0, landmark.y)))
            keypoints[name] = [u, v, visibility]
            uv_points.append((u, v))
            weight = max(0.05, visibility)
            weighted_center[0] += u * weight
            weighted_center[1] += v * weight
            visibility_sum += weight

        if "left_shoulder" not in keypoints or "right_shoulder" not in keypoints or not uv_points:
            return None

        min_u = min(point[0] for point in uv_points)
        max_u = max(point[0] for point in uv_points)
        min_v = min(point[1] for point in uv_points)
        max_v = max(point[1] for point in uv_points)
        bbox = [min_u, min_v, max_u, max_v]
        center_uv = _bbox_center(bbox)
        keypoint_center_uv = [
            weighted_center[0] / max(visibility_sum, 1e-6),
            weighted_center[1] / max(visibility_sum, 1e-6),
        ]

        return {
            "detector_id": detector_id,
            "detector_mode": str(detector_mode),
            "detector_backend": str(detector_backend),
            "bbox": bbox,
            "center_uv": center_uv,
            "keypoint_center_uv": keypoint_center_uv,
            "keypoints": keypoints,
            "proposal_bbox": list(proposal_bbox) if proposal_bbox is not None else list(bbox),
            "crop_bbox": list(crop_bbox) if crop_bbox is not None else list(bbox),
            "pose_box_source": "proposal_bbox" if proposal_bbox is not None else "landmarks_bbox",
            "detection_score": float(detection_score),
            "proposal_score": float(proposal_score),
            "image_size": [image_w, image_h],
        }

    def _update_tracks(self, detections, stamp_sec, image_w, image_h):
        self._advance_tracks(stamp_sec)
        matches, unmatched_detection_ids, unmatched_track_ids = self._match_detections_to_tracks(detections, stamp_sec)

        for track_id, detection_id, match_cost in matches:
            self._apply_detection(self.tracks[track_id], detections[detection_id], stamp_sec, image_w, image_h, match_cost)

        for track_id in unmatched_track_ids:
            self._mark_missed(self.tracks[track_id], stamp_sec)

        for detection_id in unmatched_detection_ids:
            track = TrackState(self.next_track_id, detections[detection_id], stamp_sec, max(8, self.history_size))
            self.tracks[track.track_id] = track
            self.next_track_id += 1
            self._apply_detection(track, detections[detection_id], stamp_sec, image_w, image_h, match_cost=0.0, is_new_track=True)

        self._expire_tracks(stamp_sec)
        return self._build_track_payloads(stamp_sec, image_w, image_h)

    def _advance_tracks(self, stamp_sec):
        for track in self.tracks.values():
            track.age += 1
            track.frames_seen = track.age
            track.visible = False
            track.wave_event = False
            if stamp_sec - track.last_seen_stamp > self.track_timeout_sec and track.state != "lost":
                track.state = "lost"

    def _match_detections_to_tracks(self, detections, stamp_sec):
        track_ids = list(self.tracks.keys())
        eligible_track_ids = []
        match_matrix = []
        for track_id in track_ids:
            track = self.tracks[track_id]
            if stamp_sec - track.last_seen_stamp > self.track_lost_timeout_sec:
                continue
            row = []
            for detection in detections:
                match_info = self._compute_match_cost(track, detection)
                row.append(match_info["cost"] if match_info["matched"] else None)
            eligible_track_ids.append(track_id)
            match_matrix.append(row)

        assignments = self._solve_assignment(match_matrix)
        matches = []
        matched_track_ids = set()
        matched_detection_ids = set()
        for row_index, detection_id in assignments:
            if detection_id < 0 or row_index >= len(eligible_track_ids) or detection_id >= len(detections):
                continue
            cost = match_matrix[row_index][detection_id]
            if cost is None:
                continue
            track_id = eligible_track_ids[row_index]
            if track_id in matched_track_ids or detection_id in matched_detection_ids:
                continue
            matched_track_ids.add(track_id)
            matched_detection_ids.add(detection_id)
            matches.append((track_id, detection_id, cost))

        unmatched_detection_ids = [idx for idx in range(len(detections)) if idx not in matched_detection_ids]
        unmatched_track_ids = [track_id for track_id in track_ids if track_id not in matched_track_ids]
        return matches, unmatched_detection_ids, unmatched_track_ids

    def _solve_assignment(self, match_matrix):
        if not match_matrix:
            return []
        row_count = len(match_matrix)
        col_count = max((len(row) for row in match_matrix), default=0)
        size = max(row_count, col_count)
        if size == 0:
            return []

        large_cost = 1000.0
        dummy_cost = 1.05
        cost_matrix = []
        for row in match_matrix:
            normalized = [large_cost if value is None else float(value) for value in row]
            normalized.extend([dummy_cost] * (size - len(normalized)))
            cost_matrix.append(normalized[:size])
        while len(cost_matrix) < size:
            cost_matrix.append([dummy_cost] * size)

        assignment = self._hungarian(cost_matrix)
        return [(row_index, col_index) for row_index, col_index in assignment if row_index < row_count]

    def _hungarian(self, cost_matrix):
        size = len(cost_matrix)
        u = [0.0] * (size + 1)
        v = [0.0] * (size + 1)
        p = [0] * (size + 1)
        way = [0] * (size + 1)

        for i in range(1, size + 1):
            p[0] = i
            j0 = 0
            minv = [float("inf")] * (size + 1)
            used = [False] * (size + 1)
            while True:
                used[j0] = True
                i0 = p[j0]
                delta = float("inf")
                j1 = 0
                for j in range(1, size + 1):
                    if used[j]:
                        continue
                    cur = cost_matrix[i0 - 1][j - 1] - u[i0] - v[j]
                    if cur < minv[j]:
                        minv[j] = cur
                        way[j] = j0
                    if minv[j] < delta:
                        delta = minv[j]
                        j1 = j
                for j in range(size + 1):
                    if used[j]:
                        u[p[j]] += delta
                        v[j] -= delta
                    else:
                        minv[j] -= delta
                j0 = j1
                if p[j0] == 0:
                    break
            while True:
                j1 = way[j0]
                p[j0] = p[j1]
                j0 = j1
                if j0 == 0:
                    break

        assignment = []
        for j in range(1, size + 1):
            if p[j] != 0:
                assignment.append((p[j] - 1, j - 1))
        return assignment

    def _compute_match_cost(self, track, detection):
        center_distance = _distance(track.smoothed_center, detection["center_uv"])
        keypoint_distance = _distance(track.last_keypoint_center, detection["keypoint_center_uv"])
        current_area = _bbox_area(track.smoothed_bbox)
        detection_area = _bbox_area(detection["bbox"])
        size_delta = abs(detection_area - current_area) / max(current_area, detection_area, 1e-6)
        iou = _bbox_iou(track.smoothed_bbox, detection["bbox"])
        proposal_bbox = detection.get("proposal_bbox")
        if proposal_bbox is not None and getattr(track, "proposal_bbox", None) is not None:
            iou = max(iou, _bbox_iou(track.proposal_bbox, proposal_bbox))

        distance_gate = center_distance <= self.track_match_distance_thresh
        iou_gate = iou >= self.track_iou_thresh
        relaxed_center_gate = center_distance <= self.track_match_distance_thresh * 0.55
        if not distance_gate and not iou_gate:
            return {"matched": False}
        if track.state == "lost" and not (distance_gate or relaxed_center_gate):
            return {"matched": False}

        cost = (
            0.45 * min(1.0, center_distance / max(self.track_match_distance_thresh, 1e-6))
            + 0.20 * size_delta
            + 0.20 * (1.0 - iou)
            + 0.15 * min(1.0, keypoint_distance / max(self.track_match_distance_thresh, 1e-6))
        )
        return {
            "matched": cost <= 1.0,
            "cost": cost,
            "center_distance": center_distance,
            "size_delta": size_delta,
            "iou": iou,
            "keypoint_distance": keypoint_distance,
        }

    def _apply_detection(self, track, detection, stamp_sec, image_w, image_h, match_cost, is_new_track=False):
        track.hits += 0 if is_new_track else 1
        track.last_seen_stamp = stamp_sec
        track.miss_count = 0
        track.visible = True
        track.recent_visible_streak += 1
        track.last_center = list(detection["center_uv"])
        track.last_bbox = list(detection["bbox"])
        track.last_keypoint_center = list(detection["keypoint_center_uv"])
        track.smoothed_center = _ema_vector(track.smoothed_center, detection["center_uv"], self.center_ema_alpha)
        track.smoothed_bbox = _ema_vector(track.smoothed_bbox, detection["bbox"], self.bbox_ema_alpha)
        track.keypoints = detection["keypoints"]
        track.detector_mode = str(detection.get("detector_mode", track.detector_mode))
        track.detector_backend = str(detection.get("detector_backend", track.detector_backend))
        track.proposal_bbox = list(detection.get("proposal_bbox", detection["bbox"]))
        track.crop_bbox = list(detection.get("crop_bbox", detection["bbox"]))
        track.detection_score = float(detection.get("detection_score", track.detection_score))
        track.proposal_score = float(detection.get("proposal_score", track.proposal_score))
        track.last_match_cost = match_cost

        wave_metrics = self._compute_wave_metrics(track, stamp_sec)
        track.wave_score = float(wave_metrics["wave_score"])
        track.smoothed_wave_score = (1.0 - self.score_ema_alpha) * float(track.smoothed_wave_score) + self.score_ema_alpha * track.wave_score
        track.event_confidence = float(wave_metrics["event_confidence"])
        track.wave_event = bool(wave_metrics["wave_event"])
        track.is_waving = bool(wave_metrics["is_waving"])
        track.hand_above_shoulder = bool(wave_metrics["hand_above_shoulder"])
        track.lateral_amplitude = float(wave_metrics["lateral_amplitude"])
        track.hand_raised_frames = int(wave_metrics["hand_raised_frames"])
        track.wave_flip_count = int(wave_metrics["flip_count"])
        track.wave_amplitude_norm = float(wave_metrics["wave_amplitude_norm"])
        track.wave_phase = str(wave_metrics["wave_phase"])
        track.wave_side = str(wave_metrics["wave_side"])
        if track.wave_event:
            track.last_wave_event_time = stamp_sec
            track.cooldown_until = stamp_sec + self.wave_cooldown_sec
        if track.is_waving:
            track.consecutive_wave_frames += 1
        else:
            track.consecutive_wave_frames = max(0, track.consecutive_wave_frames - 1)

        if track.hits >= self.track_activate_hits:
            track.state = "active"
        elif track.state != "lost":
            track.state = "tentative"

    def _mark_missed(self, track, stamp_sec):
        track.miss_count += 1
        track.visible = False
        track.wave_event = False
        track.recent_visible_streak = 0
        track.event_confidence *= 0.9
        if track.consecutive_wave_frames > 0:
            track.consecutive_wave_frames = max(0, track.consecutive_wave_frames - 1)
        if stamp_sec - track.last_seen_stamp > self.track_timeout_sec or track.miss_count > 0:
            track.state = "lost"

    def _expire_tracks(self, stamp_sec):
        expired = []
        for track_id, track in self.tracks.items():
            too_many_misses = track.miss_count > self.track_max_miss_count
            too_old = stamp_sec - track.last_seen_stamp > self.track_lost_timeout_sec
            if too_many_misses or too_old:
                expired.append(track_id)
        for track_id in expired:
            del self.tracks[track_id]

    def _compute_wave_metrics(self, track, stamp_sec):
        keypoints = track.keypoints
        side_info = self._select_wave_side(keypoints)
        wave_side = side_info["side"]
        hand_above_shoulder = side_info["raised"]

        if wave_side:
            shoulder = keypoints.get(wave_side + "_shoulder")
            elbow = keypoints.get(wave_side + "_elbow", shoulder)
            wrist = keypoints.get(wave_side + "_wrist")
            norm_scale = self._wave_normalization_scale(keypoints, track)
            track.wrist_history.append(wrist)
            track.wrist_x_hist.append(float(wrist[0]))
            track.wrist_y_hist.append(float(wrist[1]))
            track.shoulder_x_hist.append(float(shoulder[0]))
            track.shoulder_y_hist.append(float(shoulder[1]))
            track.elbow_x_hist.append(float(elbow[0]))
            track.elbow_y_hist.append(float(elbow[1]))
            track.timestamp_hist.append(float(stamp_sec))
            track.raised_hist.append(bool(hand_above_shoulder))
            track.x_norm_hist.append((float(wrist[0]) - float(shoulder[0])) / max(norm_scale, 1e-3))

        self._trim_wave_history(track, stamp_sec)

        x_norm_values = list(track.x_norm_hist)
        raised_values = list(track.raised_hist)
        hand_raised_frames = sum(1 for value in raised_values if value)
        amplitude = max(x_norm_values) - min(x_norm_values) if len(x_norm_values) >= 2 else 0.0
        flip_count = self._count_wave_flips(x_norm_values)

        raise_score = min(1.0, hand_raised_frames / max(1.0, float(self.min_raise_frames)))
        amplitude_score = min(1.0, amplitude / max(self.min_wave_amplitude_norm, 1e-3))
        flip_score = min(1.0, flip_count / max(1.0, float(self.min_wave_flips)))
        visibility_score = min(1.0, len(x_norm_values) / max(1.0, float(self.min_raise_frames + 1)))
        wave_score = 0.35 * raise_score + 0.30 * amplitude_score + 0.25 * flip_score + 0.10 * visibility_score

        cooldown_active = stamp_sec < track.cooldown_until
        event_ready = (
            hand_raised_frames >= self.min_raise_frames
            and amplitude >= self.min_wave_amplitude_norm
            and flip_count >= self.min_wave_flips
            and wave_score >= self.wave_event_threshold
        )
        wave_event = bool(event_ready and not cooldown_active)

        if cooldown_active:
            wave_phase = "cooldown"
        elif event_ready:
            wave_phase = "waving"
        elif hand_raised_frames >= self.min_raise_frames:
            wave_phase = "raised"
        else:
            wave_phase = "idle"

        event_confidence = min(1.0, max(wave_score, 0.55 * amplitude_score + 0.45 * flip_score))
        is_waving = bool(event_ready)

        return {
            "wave_score": wave_score,
            "event_confidence": event_confidence,
            "wave_event": wave_event,
            "is_waving": is_waving,
            "hand_above_shoulder": hand_above_shoulder,
            "hand_raised_frames": hand_raised_frames,
            "flip_count": flip_count,
            "wave_amplitude_norm": amplitude,
            "lateral_amplitude": amplitude,
            "wave_phase": wave_phase,
            "wave_side": wave_side,
        }

    def _select_wave_side(self, keypoints):
        candidates = []
        for side in ("left", "right"):
            wrist = keypoints.get(side + "_wrist")
            shoulder = keypoints.get(side + "_shoulder")
            elbow = keypoints.get(side + "_elbow", shoulder)
            if wrist is None or shoulder is None:
                continue
            visibility = min(float(wrist[2]), float(shoulder[2]))
            raised = (float(shoulder[1]) - float(wrist[1])) >= self.hand_above_shoulder_thresh
            elbow_support = elbow is None or float(wrist[1]) <= float(elbow[1]) + self.hand_above_shoulder_thresh
            raise_margin = float(shoulder[1]) - float(wrist[1])
            candidates.append(
                {
                    "side": side,
                    "wrist": wrist,
                    "shoulder": shoulder,
                    "elbow": elbow,
                    "visibility": visibility,
                    "raised": bool(raised and elbow_support),
                    "raise_margin": raise_margin,
                }
            )
        if not candidates:
            return {"side": "", "raised": False}
        candidates.sort(key=lambda item: (item["raised"], item["raise_margin"], item["visibility"]), reverse=True)
        return {"side": candidates[0]["side"], "raised": candidates[0]["raised"]}

    def _wave_normalization_scale(self, keypoints, track):
        left_shoulder = keypoints.get("left_shoulder")
        right_shoulder = keypoints.get("right_shoulder")
        shoulder_span = 0.0
        if left_shoulder is not None and right_shoulder is not None:
            shoulder_span = abs(float(left_shoulder[0]) - float(right_shoulder[0]))
        bbox_width = max(0.05, float(track.last_bbox[2]) - float(track.last_bbox[0]))
        return max(shoulder_span, bbox_width * 0.35, 1e-3)

    def _trim_wave_history(self, track, stamp_sec):
        while track.timestamp_hist and stamp_sec - float(track.timestamp_hist[0]) > self.wave_window_sec:
            track.timestamp_hist.popleft()
            if track.wrist_history:
                track.wrist_history.popleft()
            if track.wrist_x_hist:
                track.wrist_x_hist.popleft()
            if track.wrist_y_hist:
                track.wrist_y_hist.popleft()
            if track.shoulder_x_hist:
                track.shoulder_x_hist.popleft()
            if track.shoulder_y_hist:
                track.shoulder_y_hist.popleft()
            if track.elbow_x_hist:
                track.elbow_x_hist.popleft()
            if track.elbow_y_hist:
                track.elbow_y_hist.popleft()
            if track.raised_hist:
                track.raised_hist.popleft()
            if track.x_norm_hist:
                track.x_norm_hist.popleft()

    def _count_wave_flips(self, x_norm_values):
        if len(x_norm_values) < 3:
            return 0
        flip_count = 0
        last_sign = 0
        for index in range(1, len(x_norm_values)):
            delta = float(x_norm_values[index]) - float(x_norm_values[index - 1])
            if abs(delta) < 0.02:
                continue
            sign = 1 if delta > 0.0 else -1
            if last_sign != 0 and sign != last_sign:
                flip_count += 1
            last_sign = sign
        return flip_count

    def _stability_score(self, track):
        hit_ratio = min(1.0, float(track.hits) / max(1.0, float(track.frames_seen)))
        active_bonus = 1.0 if track.state == "active" else (0.5 if track.state == "lost" else 0.2)
        event_bonus = 1.0 if track.wave_event else min(1.0, float(track.event_confidence))
        return 0.30 * hit_ratio + 0.40 * max(track.smoothed_wave_score, event_bonus) + 0.30 * active_bonus

    def _build_track_payloads(self, stamp_sec, image_w, image_h):
        payload_tracks = []
        for track in sorted(self.tracks.values(), key=lambda item: item.track_id):
            payload_tracks.append(
                {
                    "track_id": track.track_id,
                    "detector_mode": track.detector_mode,
                    "detector_backend": track.detector_backend,
                    "age": track.age,
                    "hits": track.hits,
                    "frames_seen": track.frames_seen,
                    "miss_count": track.miss_count,
                    "last_seen_stamp": track.last_seen_stamp,
                    "visible": bool(track.visible),
                    "state": track.state,
                    "active": bool(track.state == "active"),
                    "bbox": [round(value, 4) for value in track.last_bbox],
                    "proposal_bbox": [round(value, 4) for value in track.proposal_bbox],
                    "crop_bbox": [round(value, 4) for value in track.crop_bbox],
                    "last_bbox": [round(value, 4) for value in track.last_bbox],
                    "smoothed_bbox": [round(value, 4) for value in track.smoothed_bbox],
                    "center_uv": [round(value, 4) for value in track.last_center],
                    "last_center": [round(value, 4) for value in track.last_center],
                    "smoothed_center": [round(value, 4) for value in track.smoothed_center],
                    "keypoint_center_uv": [round(value, 4) for value in track.last_keypoint_center],
                    "keypoints": track.keypoints,
                    "wave_score": round(track.wave_score, 4),
                    "smoothed_wave_score": round(track.smoothed_wave_score, 4),
                    "event_confidence": round(track.event_confidence, 4),
                    "wave_event": bool(track.wave_event),
                    "wave_event_time": round(track.last_wave_event_time, 4),
                    "is_waving": bool(track.is_waving),
                    "consecutive_wave_frames": track.consecutive_wave_frames,
                    "hand_above_shoulder": bool(track.hand_above_shoulder),
                    "hand_raised_frames": track.hand_raised_frames,
                    "wave_flip_count": track.wave_flip_count,
                    "wave_amplitude_norm": round(float(track.wave_amplitude_norm), 4),
                    "lateral_amplitude": round(float(track.lateral_amplitude), 4),
                    "wave_phase": track.wave_phase,
                    "wave_side": track.wave_side,
                    "cooldown_until": round(track.cooldown_until, 4),
                    "cooldown_remaining": round(max(0.0, track.cooldown_until - stamp_sec), 4),
                    "stability_score": round(self._stability_score(track), 4),
                    "detection_score": round(float(track.detection_score), 4),
                    "proposal_score": round(float(track.proposal_score), 4),
                    "recent_visible": bool(stamp_sec - track.last_seen_stamp <= self.track_timeout_sec),
                    "seconds_since_seen": round(max(0.0, stamp_sec - track.last_seen_stamp), 3),
                    "last_match_cost": round(track.last_match_cost, 4),
                    "image_size": [image_w, image_h],
                }
            )
        return payload_tracks

    def _publish_candidates(self, image_msg, tracks, stamp_sec):
        if self.publish_rate > 0.0 and stamp_sec - self.last_publish_sec < 1.0 / self.publish_rate:
            return

        state_counts = {"tentative": 0, "active": 0, "lost": 0}
        top2_ids = []
        for track in tracks:
            state_counts[track.get("state", "tentative")] = state_counts.get(track.get("state", "tentative"), 0) + 1
        ranked_tracks = sorted(
            tracks,
            key=lambda item: (
                float(item.get("event_confidence", 0.0)),
                float(item.get("smoothed_wave_score", 0.0)),
                float(item.get("stability_score", 0.0)),
            ),
            reverse=True,
        )
        top2_ids = [int(item.get("track_id", 0)) for item in ranked_tracks[:2]]

        payload = {
            "header": {
                "stamp": stamp_sec,
                "frame_id": image_msg.header.frame_id,
                "seq": image_msg.header.seq,
            },
            "image_topic": self.image_topic,
            "detector_mode": self.last_detector_mode_used,
            "detector_backend": self.last_detector_backend_used,
            "raw_person_boxes": self.last_raw_person_boxes,
            "pose_success_count": int(self.last_pose_success_count),
            "candidates": len(tracks),
            "tracked_count": len(tracks),
            "active_count": int(state_counts.get("active", 0)),
            "top2_ids": top2_ids,
            "tracks": tracks,
            "track_counts": state_counts,
            "mediapipe_available": self.mp is not None,
            "yolo_available": self.yolo_detector is not None,
            "use_mediapipe_hands": self.use_mediapipe_hands,
        }
        self.candidate_pub.publish(String(data=json.dumps(payload, ensure_ascii=True, sort_keys=True)))
        self.last_publish_sec = stamp_sec


if __name__ == "__main__":
    MediaPipeWaveDetectorNode().spin()
