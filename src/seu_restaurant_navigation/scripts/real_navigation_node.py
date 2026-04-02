#!/usr/bin/env python3
import math
import os

import actionlib
import rospy
import tf2_geometry_msgs
import tf2_ros
import yaml
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from seu_restaurant_common.constants import DEFAULT_TARGETS_PARAM
from seu_restaurant_common.service_names import get_service_name
from seu_restaurant_msgs.srv import NavToPose, NavToPoseResponse


FAILURE_UNKNOWN_TARGET = 10
FAILURE_BAD_REQUEST = 11
FAILURE_ACTION_UNAVAILABLE = 20
FAILURE_TIMEOUT = 21
FAILURE_MOVE_BASE_REJECTED = 22
FAILURE_MOVE_BASE_FAILED = 23
FAILURE_TF_ERROR = 30


def _normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _yaw_from_quaternion(quat):
    siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
    cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
    return math.atan2(siny_cosp, cosy_cosp)


class RealNavigationNode:
    def __init__(self):
        self.use_move_base_action = bool(rospy.get_param("~use_move_base_action", True))
        self.move_base_action_name = rospy.get_param("~move_base_action_name", "/move_base")
        self.goal_topic = rospy.get_param("~goal_topic", "/move_base_simple/goal")
        self.goal_frame = rospy.get_param("~goal_frame", "map")
        self.transform_goal_to_global_frame = bool(rospy.get_param("~transform_goal_to_global_frame", True))
        self.global_goal_frame = rospy.get_param("~global_goal_frame", self.goal_frame)
        self.robot_base_frame = rospy.get_param("~robot_base_frame", "base_link")
        self.success_dist_thresh = float(rospy.get_param("~success_dist_thresh", 0.35))
        self.success_yaw_thresh = float(rospy.get_param("~success_yaw_thresh", 0.35))
        self.timeout_sec = float(rospy.get_param("~timeout_sec", 120.0))
        self.poll_rate_hz = float(rospy.get_param("~poll_rate_hz", 5.0))
        self.named_targets_yaml = rospy.get_param("~named_targets_yaml", "")
        self.nav_targets_param = rospy.get_param("~named_targets_param", DEFAULT_TARGETS_PARAM)
        self.service_name = get_service_name("navigation", "nav_to_pose", private_param="~nav_service_name")
        self.prefer_startup_bar_pose = bool(rospy.get_param("~prefer_startup_bar_pose", True))
        self.allow_configured_bar_override = bool(rospy.get_param("~allow_configured_bar_override", False))
        self.capture_startup_bar_pose = bool(rospy.get_param("~capture_startup_bar_pose", True))
        self.startup_bar_target_names = set(rospy.get_param("~startup_bar_target_names", ["bar", "bar_pose", "bar_default"]))
        self.startup_bar_pose_timeout_sec = float(rospy.get_param("~startup_bar_pose_timeout_sec", 1.5))

        self.goal_publisher = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=1, latch=False)
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.named_targets = self._load_named_targets()
        self.startup_bar_pose = None

        if self.capture_startup_bar_pose:
            self._capture_startup_bar_pose(timeout_sec=self.startup_bar_pose_timeout_sec)

        self.move_base_client = None
        if self.use_move_base_action:
            self.move_base_client = actionlib.SimpleActionClient(self.move_base_action_name, MoveBaseAction)

        self.service = rospy.Service(self.service_name, NavToPose, self.handle_nav_to_pose)
        rospy.loginfo(
            "[real_navigation_node] Ready service=%s mode=%s targets=%d action=%s topic=%s",
            self.service_name,
            "move_base_action" if self.use_move_base_action else "simple_goal_tf",
            len(self.named_targets),
            self.move_base_action_name,
            self.goal_topic,
        )

    def handle_nav_to_pose(self, req):
        try:
            goal_pose, target_label, request_kind = self._resolve_goal(req)
            goal_pose = self._prepare_goal_pose(goal_pose, request_kind, target_label)
        except ValueError as exc:
            message = str(exc)
            rospy.logwarn("[real_navigation_node] %s", message)
            return NavToPoseResponse(success=False, failure_code=FAILURE_BAD_REQUEST, message=message)
        except RuntimeError as exc:
            message = str(exc)
            rospy.logwarn("[real_navigation_node] %s", message)
            return NavToPoseResponse(success=False, failure_code=FAILURE_TF_ERROR, message=message)

        rospy.loginfo(
            "[real_navigation_node] Request kind=%s target_name='%s' resolved='%s' request_frame=%s global_frame=%s final_frame=%s x=%.3f y=%.3f",
            request_kind,
            req.target_name,
            target_label,
            req.target_pose.header.frame_id if req.target_pose.header.frame_id else ("<named:{}>".format(req.target_name)),
            self.global_goal_frame,
            goal_pose.header.frame_id,
            goal_pose.pose.position.x,
            goal_pose.pose.position.y,
        )

        try:
            if self.use_move_base_action:
                return self._execute_move_base(goal_pose, target_label, request_kind)
            return self._execute_topic_goal(goal_pose, target_label, request_kind)
        except Exception as exc:
            rospy.logerr("[real_navigation_node] Navigation exception for '%s': %s", target_label, exc)
            return NavToPoseResponse(success=False, failure_code=FAILURE_MOVE_BASE_FAILED, message=str(exc))

    def _resolve_goal(self, req):
        if req.use_named_target:
            target_name = (req.target_name or "").strip()
            if not target_name:
                raise ValueError("empty target_name for named target request")
            return self._resolve_named_target_pose(target_name)

        if not req.target_pose.header.frame_id:
            raise ValueError("target_pose.header.frame_id is required when use_named_target is false")
        return self._copy_pose(req.target_pose), req.target_pose.header.frame_id, "dynamic_pose"

    def _resolve_named_target_pose(self, target_name):
        configured_pose = self.named_targets.get(target_name)

        if self._is_bar_target_name(target_name):
            if self.startup_bar_pose is None and self.capture_startup_bar_pose:
                self._capture_startup_bar_pose(timeout_sec=0.5)

            if self.prefer_startup_bar_pose and self.startup_bar_pose is not None and not self.allow_configured_bar_override:
                rospy.loginfo(
                    "[real_navigation_node] target_name='%s' using startup-captured bar pose frame=%s",
                    target_name,
                    self.startup_bar_pose.header.frame_id,
                )
                return self._copy_pose(self.startup_bar_pose), target_name, "named_target_startup_bar"

            if configured_pose is not None:
                rospy.loginfo("[real_navigation_node] target_name='%s' using configured named target", target_name)
                return self._copy_pose(configured_pose), target_name, "named_target"

            if self.startup_bar_pose is not None:
                rospy.loginfo(
                    "[real_navigation_node] target_name='%s' using startup-captured bar pose (configured missing)",
                    target_name,
                )
                return self._copy_pose(self.startup_bar_pose), target_name, "named_target_startup_bar"

        if configured_pose is None:
            known = ", ".join(sorted(self.named_targets.keys()))
            raise ValueError(
                "unknown target_name '{}'; available targets: {}".format(target_name, known or "<none>")
            )
        return self._copy_pose(configured_pose), target_name, "named_target"

    def _is_bar_target_name(self, target_name):
        return target_name in self.startup_bar_target_names

    def _capture_startup_bar_pose(self, timeout_sec):
        deadline = rospy.Time.now() + rospy.Duration(max(0.0, timeout_sec))
        while not rospy.is_shutdown():
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.global_goal_frame,
                    self.robot_base_frame,
                    rospy.Time(0),
                    rospy.Duration(0.3),
                )
                pose = PoseStamped()
                pose.header.stamp = rospy.Time.now()
                pose.header.frame_id = self.global_goal_frame
                pose.pose.position.x = transform.transform.translation.x
                pose.pose.position.y = transform.transform.translation.y
                pose.pose.position.z = transform.transform.translation.z
                pose.pose.orientation = transform.transform.rotation
                self.startup_bar_pose = pose
                rospy.loginfo(
                    "[real_navigation_node] Captured startup bar pose frame=%s x=%.3f y=%.3f",
                    pose.header.frame_id,
                    pose.pose.position.x,
                    pose.pose.position.y,
                )
                return True
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                if rospy.Time.now() >= deadline:
                    break
                rospy.sleep(0.05)
        rospy.logwarn(
            "[real_navigation_node] Failed to capture startup bar pose in frame=%s from base=%s within %.2fs",
            self.global_goal_frame,
            self.robot_base_frame,
            timeout_sec,
        )
        return False

    def _execute_move_base(self, goal_pose, target_label, request_kind):
        wait_timeout = rospy.Duration(min(self.timeout_sec, 5.0))
        if not self.move_base_client.wait_for_server(wait_timeout):
            message = "move_base action server '{}' not available for {} '{}'".format(self.move_base_action_name, request_kind, target_label)
            rospy.logerr("[real_navigation_node] %s", message)
            return NavToPoseResponse(success=False, failure_code=FAILURE_ACTION_UNAVAILABLE, message=message)

        goal = MoveBaseGoal()
        goal.target_pose = goal_pose
        self.move_base_client.send_goal(goal)
        finished = self.move_base_client.wait_for_result(rospy.Duration(self.timeout_sec))
        if not finished:
            self.move_base_client.cancel_goal()
            message = "navigation timeout after {:.1f}s for {} '{}'".format(self.timeout_sec, request_kind, target_label)
            rospy.logwarn("[real_navigation_node] %s", message)
            return NavToPoseResponse(success=False, failure_code=FAILURE_TIMEOUT, message=message)

        state = self.move_base_client.get_state()
        if state == GoalStatus.SUCCEEDED:
            message = "navigation succeeded for {} '{}'".format(request_kind, target_label)
            rospy.loginfo("[real_navigation_node] %s", message)
            return NavToPoseResponse(success=True, failure_code=0, message=message)

        status_text = self.move_base_client.get_goal_status_text() or GoalStatus.to_string(state)
        failure_code = FAILURE_MOVE_BASE_REJECTED if state in (GoalStatus.REJECTED, GoalStatus.PREEMPTED, GoalStatus.RECALLED) else FAILURE_MOVE_BASE_FAILED
        message = "navigation failed for {} '{}' state={} detail={}".format(request_kind, target_label, GoalStatus.to_string(state), status_text)
        rospy.logwarn("[real_navigation_node] %s", message)
        return NavToPoseResponse(success=False, failure_code=failure_code, message=message)

    def _execute_topic_goal(self, goal_pose, target_label, request_kind):
        self.goal_publisher.publish(goal_pose)
        rospy.loginfo(
            "[real_navigation_node] Published %s goal '%s' frame=%s x=%.3f y=%.3f to %s",
            request_kind,
            target_label,
            goal_pose.header.frame_id,
            goal_pose.pose.position.x,
            goal_pose.pose.position.y,
            self.goal_topic,
        )

        deadline = rospy.Time.now() + rospy.Duration(self.timeout_sec)
        rate = rospy.Rate(max(self.poll_rate_hz, 1.0))
        while not rospy.is_shutdown():
            if rospy.Time.now() > deadline:
                message = "navigation timeout after {:.1f}s for {} '{}'".format(self.timeout_sec, request_kind, target_label)
                rospy.logwarn("[real_navigation_node] %s", message)
                return NavToPoseResponse(success=False, failure_code=FAILURE_TIMEOUT, message=message)

            try:
                arrived, detail = self._check_goal_reached(goal_pose)
            except RuntimeError as exc:
                message = str(exc)
                rospy.logwarn("[real_navigation_node] %s", message)
                return NavToPoseResponse(success=False, failure_code=FAILURE_TF_ERROR, message=message)
            if arrived:
                message = "navigation reached {} '{}' ({})".format(request_kind, target_label, detail)
                rospy.loginfo("[real_navigation_node] %s", message)
                return NavToPoseResponse(success=True, failure_code=0, message=message)
            rate.sleep()

        return NavToPoseResponse(success=False, failure_code=FAILURE_MOVE_BASE_FAILED, message="navigation interrupted")

    def _check_goal_reached(self, goal_pose):
        try:
            transform = self.tf_buffer.lookup_transform(
                goal_pose.header.frame_id,
                self.robot_base_frame,
                rospy.Time(0),
                rospy.Duration(0.3),
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exc:
            raise RuntimeError("failed to lookup transform {} -> {}: {}".format(goal_pose.header.frame_id, self.robot_base_frame, exc))

        dx = goal_pose.pose.position.x - transform.transform.translation.x
        dy = goal_pose.pose.position.y - transform.transform.translation.y
        dist = math.hypot(dx, dy)

        robot_yaw = _yaw_from_quaternion(transform.transform.rotation)
        goal_yaw = _yaw_from_quaternion(goal_pose.pose.orientation)
        yaw_error = abs(_normalize_angle(goal_yaw - robot_yaw))

        arrived = dist <= self.success_dist_thresh and yaw_error <= self.success_yaw_thresh
        detail = "dist={:.3f} yaw_err={:.3f}".format(dist, yaw_error)
        return arrived, detail

    def _load_named_targets(self):
        targets = {}
        raw_targets = rospy.get_param(self.nav_targets_param, {})
        if isinstance(raw_targets, dict) and raw_targets:
            targets.update(raw_targets)

        if self.named_targets_yaml and os.path.isfile(self.named_targets_yaml):
            file_targets = self._read_targets_yaml(self.named_targets_yaml)
            targets.update(file_targets)
        elif self.named_targets_yaml:
            rospy.logwarn("[real_navigation_node] named_targets_yaml not found: %s", self.named_targets_yaml)

        resolved = {}
        for target_name, cfg in targets.items():
            try:
                pose, aliases = self._pose_from_config(cfg)
                resolved[target_name] = pose
                for alias in aliases:
                    resolved[alias] = self._copy_pose(pose)
            except Exception as exc:
                rospy.logwarn("[real_navigation_node] Skip target '%s': %s", target_name, exc)
        return resolved

    def _read_targets_yaml(self, yaml_path):
        with open(yaml_path, "r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}
        return data.get("restaurant", {}).get("navigation_targets", {})

    def _pose_from_config(self, cfg):
        if not isinstance(cfg, dict):
            raise ValueError("target config must be a mapping")

        pose = PoseStamped()
        pose.header.frame_id = cfg.get("frame_id", self.goal_frame)
        pose.header.stamp = rospy.Time.now()

        position = cfg.get("position", {})
        orientation = cfg.get("orientation", {})

        pose.pose.position.x = float(position.get("x", 0.0))
        pose.pose.position.y = float(position.get("y", 0.0))
        pose.pose.position.z = float(position.get("z", 0.0))
        pose.pose.orientation.x = float(orientation.get("x", 0.0))
        pose.pose.orientation.y = float(orientation.get("y", 0.0))
        pose.pose.orientation.z = float(orientation.get("z", 0.0))
        pose.pose.orientation.w = float(orientation.get("w", 1.0))

        aliases = [alias for alias in cfg.get("aliases", []) if alias]
        return pose, aliases

    def _copy_pose(self, pose):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = pose.header.frame_id or self.goal_frame
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.pose = pose.pose
        return goal_pose

    def _prepare_goal_pose(self, goal_pose, request_kind, target_label):
        if not self.transform_goal_to_global_frame:
            rospy.loginfo(
                "[real_navigation_node] TF conversion disabled; keep %s '%s' in frame=%s",
                request_kind,
                target_label,
                goal_pose.header.frame_id,
            )
            return goal_pose
        if not goal_pose.header.frame_id:
            raise ValueError("goal pose frame_id is empty for {} '{}'".format(request_kind, target_label))
        if goal_pose.header.frame_id == self.global_goal_frame:
            rospy.loginfo(
                "[real_navigation_node] No TF conversion needed for %s '%s'; already in global frame=%s",
                request_kind,
                target_label,
                self.global_goal_frame,
            )
            return goal_pose

        try:
            transformed = self.tf_buffer.transform(
                goal_pose,
                self.global_goal_frame,
                rospy.Duration(0.5),
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exc:
            raise RuntimeError(
                "failed to transform {} '{}' from frame '{}' to '{}': {}".format(
                    request_kind,
                    target_label,
                    goal_pose.header.frame_id,
                    self.global_goal_frame,
                    exc,
                )
            )

        transformed.header.stamp = rospy.Time.now()
        rospy.loginfo(
            "[real_navigation_node] TF conversion applied for %s '%s': %s -> %s x=%.3f y=%.3f",
            request_kind,
            target_label,
            goal_pose.header.frame_id,
            self.global_goal_frame,
            transformed.pose.position.x,
            transformed.pose.position.y,
        )
        return transformed


def main():
    rospy.init_node("real_navigation_node")
    RealNavigationNode()
    rospy.spin()


if __name__ == "__main__":
    main()
