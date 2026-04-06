#!/usr/bin/env python3
import math  # 提供角度归一化、距离计算等数学函数
import os  # 处理命名目标 YAML 文件路径与存在性判断

import actionlib  # ROS action 客户端，用于连接 move_base
import rospy  # ROS Python 客户端库
import tf2_geometry_msgs  # 启用 PoseStamped 的 tf2 变换支持，虽然未直接引用但导入后可注册类型转换
import tf2_ros  # TF2 监听与坐标变换库
import yaml  # 解析命名导航点配置文件
from actionlib_msgs.msg import GoalStatus  # move_base action 返回状态码定义
from geometry_msgs.msg import PoseStamped  # 统一使用 PoseStamped 描述导航目标位姿
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  # move_base action 类型及目标消息

from seu_restaurant_common.constants import DEFAULT_TARGETS_PARAM  # 默认的命名目标参数键
from seu_restaurant_common.service_names import get_service_name  # 统一服务名解析工具
from seu_restaurant_msgs.srv import NavToPose, NavToPoseResponse  # 导航服务请求与响应类型


FAILURE_UNKNOWN_TARGET = 10  # 预留的未知目标错误码，当前实现主要通过 BAD_REQUEST 承载
FAILURE_BAD_REQUEST = 11  # 请求格式错误，例如目标名为空或动态目标缺少 frame_id
FAILURE_ACTION_UNAVAILABLE = 20  # move_base action server 不可用
FAILURE_TIMEOUT = 21  # 导航在设定时间内未完成
FAILURE_MOVE_BASE_REJECTED = 22  # action 目标被拒绝、抢占或撤回
FAILURE_MOVE_BASE_FAILED = 23  # 导航执行失败或被中断
FAILURE_TF_ERROR = 30  # TF 查询或坐标变换出错


def _normalize_angle(angle):  # 把任意弧度角规整到 [-pi, pi]，便于正确比较朝向误差
    while angle > math.pi:
        angle -= 2.0 * math.pi  # 超过 pi 时向回减一个整圆
    while angle < -math.pi:
        angle += 2.0 * math.pi  # 小于 -pi 时向前加一个整圆
    return angle  # 返回归一化后的角度


def _yaw_from_quaternion(quat):  # 从四元数中提取平面导航最关心的偏航角 yaw
    siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)  # atan2 分子项
    cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)  # atan2 分母项
    return math.atan2(siny_cosp, cosy_cosp)  # 返回机器人或目标的朝向角


def _quaternion_from_yaw(yaw):  # 根据平面偏航角构造四元数，供二维导航目标使用
    half_yaw = 0.5 * yaw
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


class RealNavigationNode:  # 真实导航节点：向上层暴露统一 nav_to_pose 服务，对下适配 move_base 或 simple goal
    def __init__(self):
        self.use_move_base_action = bool(rospy.get_param("~use_move_base_action", True))  # 是否通过 move_base action 执行导航
        self.move_base_action_name = rospy.get_param("~move_base_action_name", "/move_base")  # move_base action 名称
        self.goal_topic = rospy.get_param("~goal_topic", "/move_base_simple/goal")  # simple goal 模式下的目标发布主题
        self.goal_frame = rospy.get_param("~goal_frame", "map")  # 默认目标位姿所属坐标系
        self.transform_goal_to_global_frame = bool(rospy.get_param("~transform_goal_to_global_frame", True))  # 是否把目标统一变换到全局坐标系
        self.global_goal_frame = rospy.get_param("~global_goal_frame", self.goal_frame)  # 最终导航执行所用的全局坐标系
        self.robot_base_frame = rospy.get_param("~robot_base_frame", "base_link")  # 机器人本体坐标系，用于 TF 查询当前位置
        self.success_dist_thresh = float(rospy.get_param("~success_dist_thresh", 0.35))  # 位置到达阈值，单位米
        self.success_yaw_thresh = float(rospy.get_param("~success_yaw_thresh", 0.35))  # 朝向到达阈值，单位弧度
        self.timeout_sec = float(rospy.get_param("~timeout_sec", 120.0))  # 单次导航允许持续的最长时间
        self.poll_rate_hz = float(rospy.get_param("~poll_rate_hz", 5.0))  # simple goal 模式下轮询到达状态的频率
        self.named_targets_yaml = rospy.get_param("~named_targets_yaml", "")  # 命名目标 YAML 文件路径
        self.nav_targets_param = rospy.get_param("~named_targets_param", DEFAULT_TARGETS_PARAM)  # 参数服务器中命名目标的键名
        self.service_name = get_service_name("navigation", "nav_to_pose", private_param="~nav_service_name")  # 对外导航服务名
        self.prefer_startup_bar_pose = bool(rospy.get_param("~prefer_startup_bar_pose", True))  # 去吧台时是否优先使用启动抓取位姿
        self.allow_configured_bar_override = bool(rospy.get_param("~allow_configured_bar_override", False))  # 是否允许配置文件中的吧台点覆盖启动抓取位姿
        self.capture_startup_bar_pose = bool(rospy.get_param("~capture_startup_bar_pose", True))  # 启动时是否自动记录当前机器人位姿为吧台参考点
        self.startup_bar_target_names = set(rospy.get_param("~startup_bar_target_names", ["bar", "bar_pose", "bar_default"]))  # 哪些命名目标被视作“吧台”
        self.startup_bar_pose_timeout_sec = float(rospy.get_param("~startup_bar_pose_timeout_sec", 1.5))  # 启动抓取吧台位姿的超时时间
        self.startup_bar_yaw_offset = float(rospy.get_param("~startup_bar_yaw_offset", 0.0))  # 启动抓取吧台位姿时额外叠加的偏航角偏置

        self.goal_publisher = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=1, latch=False)  # simple goal 模式下向导航栈发布目标
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))  # TF 缓冲区，保存最近一段时间的坐标变换
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)  # 监听整个系统 TF 树
        self.named_targets = self._load_named_targets()  # 启动时加载所有命名目标与别名
        self.startup_bar_pose = None  # 缓存启动时记录的吧台位姿

        if self.capture_startup_bar_pose:
            self._capture_startup_bar_pose(timeout_sec=self.startup_bar_pose_timeout_sec)  # 节点启动即尝试记录当前位置，供后续“回吧台”使用

        self.move_base_client = None  # action 模式下的客户端句柄，simple goal 模式保持为空
        if self.use_move_base_action:
            self.move_base_client = actionlib.SimpleActionClient(self.move_base_action_name, MoveBaseAction)  # 连接标准 move_base action server

        self.service = rospy.Service(self.service_name, NavToPose, self.handle_nav_to_pose)  # 注册导航服务，对外接收“去某个位姿/命名点”的请求
        rospy.loginfo(
            "[real_navigation_node] Ready service=%s mode=%s targets=%d action=%s topic=%s",
            self.service_name,
            "move_base_action" if self.use_move_base_action else "simple_goal_tf",
            len(self.named_targets),
            self.move_base_action_name,
            self.goal_topic,
        )  # 输出当前模式、可用目标数量和底层执行通道，便于排查配置是否生效

    def handle_nav_to_pose(self, req):  # 导航服务主入口：解析请求、准备目标、执行导航并返回状态
        try:
            goal_pose, target_label, request_kind = self._resolve_goal(req)  # 先确定目标来自命名点还是动态位姿
            goal_pose = self._prepare_goal_pose(goal_pose, request_kind, target_label)  # 按配置决定是否做 TF 坐标变换
        except ValueError as exc:
            message = str(exc)  # 请求内容本身不合法，例如目标名空或 frame_id 缺失
            rospy.logwarn("[real_navigation_node] %s", message)
            return NavToPoseResponse(success=False, failure_code=FAILURE_BAD_REQUEST, message=message)  # 返回请求错误码给上层
        except RuntimeError as exc:
            message = str(exc)  # 这里主要承载 TF 查询或变换失败
            rospy.logwarn("[real_navigation_node] %s", message)
            return NavToPoseResponse(success=False, failure_code=FAILURE_TF_ERROR, message=message)  # 返回 TF 错误码

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
        )  # 打印本次请求最终落地后的目标信息，便于核对命名点解析和 TF 变换是否正确

        try:
            if self.use_move_base_action:
                return self._execute_move_base(goal_pose, target_label, request_kind)  # 走标准 action 导航链路
            return self._execute_topic_goal(goal_pose, target_label, request_kind)  # 走 simple goal + TF 到达判定链路
        except Exception as exc:
            rospy.logerr("[real_navigation_node] Navigation exception for '%s': %s", target_label, exc)  # 捕获未分类异常，避免服务端崩溃
            return NavToPoseResponse(success=False, failure_code=FAILURE_MOVE_BASE_FAILED, message=str(exc))  # 统一返回执行失败

    def _resolve_goal(self, req):  # 解析请求里的目标来源，统一转成 PoseStamped + 描述标签
        if req.use_named_target:
            target_name = (req.target_name or "").strip()  # 清理命名目标字符串，避免两侧空白导致匹配失败
            if not target_name:
                raise ValueError("empty target_name for named target request")  # 命名目标模式下目标名不能为空
            return self._resolve_named_target_pose(target_name)  # 从已加载的命名目标表中解析位姿

        if not req.target_pose.header.frame_id:
            raise ValueError("target_pose.header.frame_id is required when use_named_target is false")  # 动态位姿模式必须明确给出坐标系
        return self._copy_pose(req.target_pose), req.target_pose.header.frame_id, "dynamic_pose"  # 复制动态位姿避免直接改写请求对象

    def _resolve_named_target_pose(self, target_name):  # 解析命名目标位姿，并对“吧台”这类特殊目标应用优先级规则
        configured_pose = self.named_targets.get(target_name)  # 先尝试从配置和别名表里直接取目标

        if self._is_bar_target_name(target_name):
            if self.startup_bar_pose is None and self.capture_startup_bar_pose:
                self._capture_startup_bar_pose(timeout_sec=0.5)  # 若启动时没抓到，则在首次请求吧台目标时再补抓一次

            if self.prefer_startup_bar_pose and self.startup_bar_pose is not None and not self.allow_configured_bar_override:
                rospy.loginfo(
                    "[real_navigation_node] target_name='%s' using startup-captured bar pose frame=%s",
                    target_name,
                    self.startup_bar_pose.header.frame_id,
                )  # 明确记录：本次吧台导航优先采用启动时位姿
                return self._copy_pose(self.startup_bar_pose), target_name, "named_target_startup_bar"  # 返回启动时记录的吧台参考点

            if configured_pose is not None:
                rospy.loginfo("[real_navigation_node] target_name='%s' using configured named target", target_name)  # 若允许则退回配置文件中的吧台点
                return self._copy_pose(configured_pose), target_name, "named_target"  # 返回配置中的命名目标位姿

            if self.startup_bar_pose is not None:
                rospy.loginfo(
                    "[real_navigation_node] target_name='%s' using startup-captured bar pose (configured missing)",
                    target_name,
                )  # 配置里没这个吧台点时，仍可回退到启动时抓到的位姿
                return self._copy_pose(self.startup_bar_pose), target_name, "named_target_startup_bar"  # 返回启动时位姿作为容错方案

        if configured_pose is None:
            known = ", ".join(sorted(self.named_targets.keys()))  # 把当前已知目标列出来，便于报错时提示用户检查配置
            raise ValueError(
                "unknown target_name '{}'; available targets: {}".format(target_name, known or "<none>")
            )  # 未知命名目标直接视为请求错误
        return self._copy_pose(configured_pose), target_name, "named_target"  # 普通命名点直接返回配置位姿

    def _is_bar_target_name(self, target_name):  # 判断目标名是否属于吧台/起始点这一特殊类别
        return target_name in self.startup_bar_target_names  # 使用集合查找，时间复杂度为 O(1)

    def _capture_startup_bar_pose(self, timeout_sec):  # 从 TF 中抓取当前机器人位姿，保存为启动吧台参考点
        deadline = rospy.Time.now() + rospy.Duration(max(0.0, timeout_sec))  # 计算最晚等待到什么时候为止
        while not rospy.is_shutdown():
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.global_goal_frame,
                    self.robot_base_frame,
                    rospy.Time(0),
                    rospy.Duration(0.3),
                )  # 查询“机器人基座在全局坐标系下的位置”
                pose = PoseStamped()  # 构造一个标准位姿对象保存当前机器人位置
                pose.header.stamp = rospy.Time.now()  # 记录抓取时刻
                pose.header.frame_id = self.global_goal_frame  # 统一保存到全局坐标系中
                pose.pose.position.x = transform.transform.translation.x  # 当前 x 坐标
                pose.pose.position.y = transform.transform.translation.y  # 当前 y 坐标
                pose.pose.position.z = transform.transform.translation.z  # 当前 z 坐标
                startup_yaw = _yaw_from_quaternion(transform.transform.rotation)  # 提取启动时机器人原始朝向
                target_yaw = _normalize_angle(startup_yaw + self.startup_bar_yaw_offset)  # 叠加期望偏置后的吧台朝向
                quat_x, quat_y, quat_z, quat_w = _quaternion_from_yaw(target_yaw)
                pose.pose.orientation.x = quat_x
                pose.pose.orientation.y = quat_y
                pose.pose.orientation.z = quat_z
                pose.pose.orientation.w = quat_w
                self.startup_bar_pose = pose  # 缓存下来，后续回吧台时可以直接使用
                rospy.loginfo(
                    "[real_navigation_node] Captured startup bar pose frame=%s x=%.3f y=%.3f yaw=%.3f yaw_offset=%.3f",
                    pose.header.frame_id,
                    pose.pose.position.x,
                    pose.pose.position.y,
                    target_yaw,
                    self.startup_bar_yaw_offset,
                )  # 输出抓取到的吧台参考点
                return True  # 抓取成功立即返回
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                if rospy.Time.now() >= deadline:
                    break  # 到达截止时间后停止重试
                rospy.sleep(0.05)  # TF 尚未稳定时短暂等待再试
        rospy.logwarn(
            "[real_navigation_node] Failed to capture startup bar pose in frame=%s from base=%s within %.2fs",
            self.global_goal_frame,
            self.robot_base_frame,
            timeout_sec,
        )  # 明确提示启动吧台位姿未能成功记录
        return False  # 抓取失败

    def _execute_move_base(self, goal_pose, target_label, request_kind):  # 通过 move_base action 执行导航
        wait_timeout = rospy.Duration(min(self.timeout_sec, 5.0))  # 等 action server 的时间不必和导航总超时完全一致
        if not self.move_base_client.wait_for_server(wait_timeout):
            message = "move_base action server '{}' not available for {} '{}'".format(self.move_base_action_name, request_kind, target_label)  # move_base 没起来时直接失败
            rospy.logerr("[real_navigation_node] %s", message)
            return NavToPoseResponse(success=False, failure_code=FAILURE_ACTION_UNAVAILABLE, message=message)  # 返回 action 不可用错误码

        goal = MoveBaseGoal()  # 构造标准 move_base 目标消息
        goal.target_pose = goal_pose  # 填入最终目标位姿
        self.move_base_client.send_goal(goal)  # 把目标发送给导航栈
        finished = self.move_base_client.wait_for_result(rospy.Duration(self.timeout_sec))  # 阻塞等待导航结束或超时
        if not finished:
            self.move_base_client.cancel_goal()  # 超时后主动取消当前目标，避免机器人继续运行
            message = "navigation timeout after {:.1f}s for {} '{}'".format(self.timeout_sec, request_kind, target_label)  # 生成超时说明
            rospy.logwarn("[real_navigation_node] %s", message)
            return NavToPoseResponse(success=False, failure_code=FAILURE_TIMEOUT, message=message)  # 返回超时失败

        state = self.move_base_client.get_state()  # 获取导航动作最终状态
        if state == GoalStatus.SUCCEEDED:
            message = "navigation succeeded for {} '{}'".format(request_kind, target_label)  # 成功时返回简洁成功说明
            rospy.loginfo("[real_navigation_node] %s", message)
            return NavToPoseResponse(success=True, failure_code=0, message=message)  # 0 代表无错误

        status_text = self.move_base_client.get_goal_status_text() or GoalStatus.to_string(state)  # 获取底层 action 的文字说明
        failure_code = FAILURE_MOVE_BASE_REJECTED if state in (GoalStatus.REJECTED, GoalStatus.PREEMPTED, GoalStatus.RECALLED) else FAILURE_MOVE_BASE_FAILED  # 按状态分类映射错误码
        message = "navigation failed for {} '{}' state={} detail={}".format(request_kind, target_label, GoalStatus.to_string(state), status_text)  # 组合失败详情，便于上层日志查看
        rospy.logwarn("[real_navigation_node] %s", message)
        return NavToPoseResponse(success=False, failure_code=failure_code, message=message)  # 返回 action 失败结果

    def _execute_topic_goal(self, goal_pose, target_label, request_kind):  # 通过 simple goal topic 发布目标，并自行轮询判断是否到达
        self.goal_publisher.publish(goal_pose)  # 向下游导航模块发布目标位姿
        rospy.loginfo(
            "[real_navigation_node] Published %s goal '%s' frame=%s x=%.3f y=%.3f to %s",
            request_kind,
            target_label,
            goal_pose.header.frame_id,
            goal_pose.pose.position.x,
            goal_pose.pose.position.y,
            self.goal_topic,
        )  # 记录已发布的目标信息，便于和 RViz/日志核对

        deadline = rospy.Time.now() + rospy.Duration(self.timeout_sec)  # 计算导航超时截止时间
        rate = rospy.Rate(max(self.poll_rate_hz, 1.0))  # 轮询频率至少为 1Hz，避免配置错误造成死循环
        while not rospy.is_shutdown():
            if rospy.Time.now() > deadline:
                message = "navigation timeout after {:.1f}s for {} '{}'".format(self.timeout_sec, request_kind, target_label)  # 发布目标后长时间未到达
                rospy.logwarn("[real_navigation_node] %s", message)
                return NavToPoseResponse(success=False, failure_code=FAILURE_TIMEOUT, message=message)  # 返回超时失败

            try:
                arrived, detail = self._check_goal_reached(goal_pose)  # 通过 TF 实时计算当前位置与目标位姿的误差
            except RuntimeError as exc:
                message = str(exc)  # 一旦 TF 查不到就无法判断是否到达
                rospy.logwarn("[real_navigation_node] %s", message)
                return NavToPoseResponse(success=False, failure_code=FAILURE_TF_ERROR, message=message)  # 返回 TF 错误码
            if arrived:
                message = "navigation reached {} '{}' ({})".format(request_kind, target_label, detail)  # 记录最终误差明细，说明为什么判定为已到达
                rospy.loginfo("[real_navigation_node] %s", message)
                return NavToPoseResponse(success=True, failure_code=0, message=message)  # 到达则返回成功
            rate.sleep()  # 未到达则等待下一轮检查

        return NavToPoseResponse(success=False, failure_code=FAILURE_MOVE_BASE_FAILED, message="navigation interrupted")  # 节点关闭或循环被打断时的兜底返回

    def _check_goal_reached(self, goal_pose):  # 通过 TF 计算当前位置和目标位姿之间的位置与朝向误差
        try:
            transform = self.tf_buffer.lookup_transform(
                goal_pose.header.frame_id,
                self.robot_base_frame,
                rospy.Time(0),
                rospy.Duration(0.3),
            )  # 查询机器人基座在目标坐标系下的当前变换
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exc:
            raise RuntimeError("failed to lookup transform {} -> {}: {}".format(goal_pose.header.frame_id, self.robot_base_frame, exc))  # 向上抛出统一 TF 错误

        dx = goal_pose.pose.position.x - transform.transform.translation.x  # 当前位置到目标点的 x 方向误差
        dy = goal_pose.pose.position.y - transform.transform.translation.y  # 当前位置到目标点的 y 方向误差
        dist = math.hypot(dx, dy)  # 平面欧氏距离误差

        robot_yaw = _yaw_from_quaternion(transform.transform.rotation)  # 机器人当前偏航角
        goal_yaw = _yaw_from_quaternion(goal_pose.pose.orientation)  # 目标偏航角
        yaw_error = abs(_normalize_angle(goal_yaw - robot_yaw))  # 两者之间的最小角度差

        arrived = dist <= self.success_dist_thresh and yaw_error <= self.success_yaw_thresh  # 位置与朝向同时满足阈值才算真正到达
        detail = "dist={:.3f} yaw_err={:.3f}".format(dist, yaw_error)  # 返回误差明细方便上层日志记录
        return arrived, detail  # 返回布尔结果和详细说明

    def _load_named_targets(self):  # 从参数服务器和 YAML 文件两处加载命名目标，并展开别名
        targets = {}  # 临时原始目标配置表
        raw_targets = rospy.get_param(self.nav_targets_param, {})  # 优先从参数服务器读取目标定义
        if isinstance(raw_targets, dict) and raw_targets:
            targets.update(raw_targets)  # 合并参数服务器中的目标配置

        if self.named_targets_yaml and os.path.isfile(self.named_targets_yaml):
            file_targets = self._read_targets_yaml(self.named_targets_yaml)  # 读取 YAML 文件中的目标配置
            targets.update(file_targets)  # 后加载内容覆盖前面同名项，便于用文件统一修正
        elif self.named_targets_yaml:
            rospy.logwarn("[real_navigation_node] named_targets_yaml not found: %s", self.named_targets_yaml)  # 文件路径配置了但实际不存在

        resolved = {}  # 最终可用的目标名到 PoseStamped 的映射表，包含别名
        for target_name, cfg in targets.items():
            try:
                pose, aliases = self._pose_from_config(cfg)  # 把单个目标配置转成标准 PoseStamped
                resolved[target_name] = pose  # 注册主目标名
                for alias in aliases:
                    resolved[alias] = self._copy_pose(pose)  # 给每个别名也登记一个独立副本，避免共用对象被修改
            except Exception as exc:
                rospy.logwarn("[real_navigation_node] Skip target '%s': %s", target_name, exc)  # 单个目标配置坏掉时仅跳过，不阻止整个节点启动
        return resolved  # 返回解析完成的命名目标表

    def _read_targets_yaml(self, yaml_path):  # 读取命名目标 YAML 文件并取出约定字段
        with open(yaml_path, "r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}  # YAML 为空时退回空字典，避免后续空引用
        return data.get("restaurant", {}).get("navigation_targets", {})  # 只提取项目定义的导航目标层级

    def _pose_from_config(self, cfg):  # 把单个目标配置字典解析为 PoseStamped 与别名列表
        if not isinstance(cfg, dict):
            raise ValueError("target config must be a mapping")  # 配置项必须是字典结构

        pose = PoseStamped()  # 构造标准位姿对象
        pose.header.frame_id = cfg.get("frame_id", self.goal_frame)  # 未指定 frame_id 时使用默认目标坐标系
        pose.header.stamp = rospy.Time.now()  # 给位姿打上当前时间戳

        position = cfg.get("position", {})  # 提取平移部分配置
        orientation = cfg.get("orientation", {})  # 提取旋转部分配置

        pose.pose.position.x = float(position.get("x", 0.0))  # 目标 x 坐标
        pose.pose.position.y = float(position.get("y", 0.0))  # 目标 y 坐标
        pose.pose.position.z = float(position.get("z", 0.0))  # 目标 z 坐标，二维导航中通常保持 0
        pose.pose.orientation.x = float(orientation.get("x", 0.0))  # 四元数 x 分量
        pose.pose.orientation.y = float(orientation.get("y", 0.0))  # 四元数 y 分量
        pose.pose.orientation.z = float(orientation.get("z", 0.0))  # 四元数 z 分量
        pose.pose.orientation.w = float(orientation.get("w", 1.0))  # 四元数 w 分量，默认单位四元数

        aliases = [alias for alias in cfg.get("aliases", []) if alias]  # 过滤空字符串，只保留有效别名
        return pose, aliases  # 返回位姿和该目标的所有别名

    def _copy_pose(self, pose):  # 复制一个 PoseStamped，避免多个引用共享同一对象
        goal_pose = PoseStamped()  # 构造新的位姿对象
        goal_pose.header.frame_id = pose.header.frame_id or self.goal_frame  # 若原对象无 frame_id，则补默认坐标系
        goal_pose.header.stamp = rospy.Time.now()  # 复制时刷新时间戳，表示这是新的当前目标
        goal_pose.pose = pose.pose  # 复制姿态与位置数据
        return goal_pose  # 返回副本

    def _prepare_goal_pose(self, goal_pose, request_kind, target_label):  # 根据配置决定是否把目标转换到统一全局坐标系
        if not self.transform_goal_to_global_frame:
            rospy.loginfo(
                "[real_navigation_node] TF conversion disabled; keep %s '%s' in frame=%s",
                request_kind,
                target_label,
                goal_pose.header.frame_id,
            )  # 显式说明本次不做 TF 变换
            return goal_pose  # 直接沿用原始目标坐标系
        if not goal_pose.header.frame_id:
            raise ValueError("goal pose frame_id is empty for {} '{}'".format(request_kind, target_label))  # frame_id 为空无法做任何导航
        if goal_pose.header.frame_id == self.global_goal_frame:
            rospy.loginfo(
                "[real_navigation_node] No TF conversion needed for %s '%s'; already in global frame=%s",
                request_kind,
                target_label,
                self.global_goal_frame,
            )  # 目标已在全局坐标系下，无需再次变换
            return goal_pose  # 直接返回原目标

        try:
            transformed = self.tf_buffer.transform(
                goal_pose,
                self.global_goal_frame,
                rospy.Duration(0.5),
            )  # 将目标从其原始坐标系变换到全局坐标系
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exc:
            raise RuntimeError(
                "failed to transform {} '{}' from frame '{}' to '{}': {}".format(
                    request_kind,
                    target_label,
                    goal_pose.header.frame_id,
                    self.global_goal_frame,
                    exc,
                )
            )  # 变换失败时上抛，由上层统一按 TF 错误处理

        transformed.header.stamp = rospy.Time.now()  # 刷新变换后的目标时间戳
        rospy.loginfo(
            "[real_navigation_node] TF conversion applied for %s '%s': %s -> %s x=%.3f y=%.3f",
            request_kind,
            target_label,
            goal_pose.header.frame_id,
            self.global_goal_frame,
            transformed.pose.position.x,
            transformed.pose.position.y,
        )  # 输出变换前后的坐标系和目标位置，便于验证 TF 是否正确
        return transformed  # 返回已统一到全局坐标系下的目标


def main():  # ROS 节点入口函数
    rospy.init_node("real_navigation_node")  # 初始化真实导航节点
    RealNavigationNode()  # 创建节点对象并注册导航服务
    rospy.spin()  # 进入事件循环，持续对外提供导航服务


if __name__ == "__main__":
    main()  # 脚本直接运行时从这里进入主逻辑
