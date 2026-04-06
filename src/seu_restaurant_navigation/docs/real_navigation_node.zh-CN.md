# `real_navigation_node.py` 中文注释说明

对应源码：`src/seu_restaurant_navigation/scripts/real_navigation_node.py`

说明：本文件是**外部中文注释文档**，用于解释导航节点的真实逻辑；**不修改原始源码**。

## 1. 文件职责

这个脚本实现了项目中的“真实导航节点”，主要职责是：

1. 对外提供统一的 ROS 导航服务 `nav_to_pose`。
2. 支持两种导航输入：
   - 命名目标点（如 `bar`、`table1`）。
   - 动态传入的 `PoseStamped` 目标位姿。
3. 支持两种执行模式：
   - 通过 `move_base` action 发送目标。
   - 通过 topic 发布目标点，再结合 TF 轮询判断是否到达。
4. 负责做目标解析、TF 坐标变换、到达判定和错误码返回。

它可以看作是“**导航能力的统一服务封装层**”。

## 2. 文件整体结构

源码可以分为 5 个部分：

1. 常量与辅助函数。
2. `RealNavigationNode` 初始化。
3. 服务入口 `handle_nav_to_pose`。
4. 目标解析、执行与到达判定。
5. 配置加载与程序入口。

## 3. 常量与辅助函数

### 3.1 失败码常量（第 19–25 行）

这些常量定义了导航失败时的分类编码：

- `FAILURE_UNKNOWN_TARGET = 10`
  - 目标点名字无法识别。
- `FAILURE_BAD_REQUEST = 11`
  - 请求格式不合法，例如空目标名、缺 frame_id。
- `FAILURE_ACTION_UNAVAILABLE = 20`
  - `move_base` action server 没起来。
- `FAILURE_TIMEOUT = 21`
  - 导航超时。
- `FAILURE_MOVE_BASE_REJECTED = 22`
  - 目标被拒绝、取消或召回。
- `FAILURE_MOVE_BASE_FAILED = 23`
  - 其他导航执行失败。
- `FAILURE_TF_ERROR = 30`
  - TF 查询或坐标变换失败。

这些失败码的意义是：让上层状态机不仅知道“失败了”，还知道“为什么失败”。

### 3.2 `_normalize_angle`（第 28–33 行）

这个函数把任意角度规整到 `[-pi, pi]` 区间。

作用：

- 角度差比较时，不能直接做普通减法。
- 比如 `179°` 和 `-179°` 实际只差 `2°`，但直接相减会得到 `358°`。
- 该函数用于保证朝向误差的计算是合理的。

### 3.3 `_yaw_from_quaternion`（第 36–39 行）

这个函数从四元数中提取平面朝向 `yaw`。

用途：

- 机器人在二维导航里，通常只关心偏航角 `yaw`。
- 在到达判定时，需要比较机器人当前朝向与目标朝向的误差。

## 4. `RealNavigationNode` 初始化逻辑

### 4.1 参数读取（第 42–63 行）

构造函数一开始读取了大量 ROS 参数，分别控制导航方式和行为细节。

#### 4.1.1 执行模式相关参数

- `self.use_move_base_action`
  - `True`：使用 action 调 `move_base`。
  - `False`：只发 topic，再自己轮询 TF 判断是否到达。
- `self.move_base_action_name`
  - action server 名称，默认 `/move_base`。
- `self.goal_topic`
  - topic 模式下发布目标的主题，默认 `/move_base_simple/goal`。

#### 4.1.2 坐标系相关参数

- `self.goal_frame`
  - 目标点默认坐标系。
- `self.transform_goal_to_global_frame`
  - 是否把所有目标统一转换到全局坐标系。
- `self.global_goal_frame`
  - 全局目标坐标系，通常是 `map`。
- `self.robot_base_frame`
  - 机器人本体坐标系，通常是 `base_link`。

#### 4.1.3 到达判定与超时参数

- `self.success_dist_thresh`
  - 位置到达阈值。
- `self.success_yaw_thresh`
  - 朝向到达阈值。
- `self.timeout_sec`
  - 单次导航最长允许持续时间。
- `self.poll_rate_hz`
  - topic 模式下检查到达状态的频率。

#### 4.1.4 命名目标配置来源

- `self.named_targets_yaml`
  - 目标点 YAML 文件路径。
- `self.nav_targets_param`
  - 参数服务器中的命名目标参数名。
- `self.service_name`
  - ROS 导航服务名，由统一服务名工具取得。

#### 4.1.5 启动时吧台位姿相关参数

这部分是该文件比较有项目特色的逻辑。

- `self.prefer_startup_bar_pose`
  - 导航到吧台时，是否优先使用“启动时捕获”的吧台位姿。
- `self.allow_configured_bar_override`
  - 即使有启动时位姿，是否允许配置文件中的吧台位姿覆盖它。
- `self.capture_startup_bar_pose`
  - 启动时是否主动从 TF 抓取机器人当前位姿，作为“吧台初始位姿”。
- `self.startup_bar_target_names`
  - 哪些目标名被视作“吧台目标”。
- `self.startup_bar_pose_timeout_sec`
  - 启动抓取吧台位姿的超时时间。

这套逻辑通常用于：机器人一开始就停在吧台附近，希望后面“回到吧台”时回到启动位置，而不是完全依赖预设地图点。

### 4.2 初始化运行组件（第 64–77 行）

- `self.goal_publisher = rospy.Publisher(...)`
  - topic 模式下的目标发布器。
- `self.tf_buffer / self.tf_listener`
  - TF 缓冲区与监听器，用于坐标变换和获取机器人当前位姿。
- `self.named_targets = self._load_named_targets()`
  - 启动时加载命名目标点。
- `self.startup_bar_pose = None`
  - 启动时吧台位姿的缓存变量。
- `if self.capture_startup_bar_pose: ...`
  - 若启用，则启动即抓取一次当前位姿。
- `self.move_base_client = actionlib.SimpleActionClient(...)`
  - 只有在 action 模式下才创建 action client。
- `self.service = rospy.Service(...)`
  - 注册导航服务。

最后打印 ready 日志，帮助确认当前导航模式、目标数、action 名称和 topic 名称。

## 5. 服务入口 `handle_nav_to_pose`（第 87–118 行）

这是对外的主入口。

### 5.1 解析请求

- `goal_pose, target_label, request_kind = self._resolve_goal(req)`
  - 先根据请求内容确定目标点。
  - 可能来自命名目标，也可能来自动态位姿。

### 5.2 做目标预处理

- `goal_pose = self._prepare_goal_pose(...)`
  - 如果需要，把目标位姿转换到统一的全局坐标系。

### 5.3 分类处理异常

- `ValueError`
  - 视为请求不合法，返回 `FAILURE_BAD_REQUEST`。
- `RuntimeError`
  - 视为 TF 相关错误，返回 `FAILURE_TF_ERROR`。

### 5.4 记录解析结果

日志里会打印：

- 请求类型。
- 原始目标名。
- 最终解析出的标签。
- 请求坐标系。
- 全局坐标系。
- 最终目标坐标系。
- 最终目标位置坐标。

### 5.5 按模式执行导航

- 如果 `self.use_move_base_action` 为真
  - 调 `_execute_move_base()`。
- 否则
  - 调 `_execute_topic_goal()`。

任何未捕获异常都统一记录为导航执行异常，并返回 `FAILURE_MOVE_BASE_FAILED`。

## 6. 目标解析逻辑

### 6.1 `_resolve_goal`（第 120–129 行）

这个函数用于判断本次请求到底是哪一种目标来源。

- 当 `req.use_named_target` 为真
  - 说明调用方给的是一个名字，比如 `bar`。
  - 这时转去 `_resolve_named_target_pose(target_name)`。
- 否则
  - 说明调用方直接给了 `PoseStamped`。
  - 此时必须要求 `target_pose.header.frame_id` 非空。

返回值是一个三元组：

1. `goal_pose`：真正要去的位姿。
2. `target_label`：用于日志和结果消息的目标标签。
3. `request_kind`：目标来源类型说明。

### 6.2 `_resolve_named_target_pose`（第 131–162 行）

这是命名目标解析的核心函数。

#### 6.2.1 先从已加载配置中取目标

- `configured_pose = self.named_targets.get(target_name)`
  - 从配置字典中查目标位姿。

#### 6.2.2 如果这是“吧台类目标”

- 先判断 `target_name` 是否属于吧台名集合。
- 如果启动时位姿还没抓到，但允许抓取，则再补抓一次。
- 如果满足以下条件：
  - 优先使用启动时吧台位姿。
  - 启动时位姿存在。
  - 不允许配置覆盖。
  - 那么就直接返回启动时抓取的位姿。

#### 6.2.3 如果吧台配置目标存在

- 则可返回配置中的命名目标。

#### 6.2.4 如果配置中没有，但启动时位姿存在

- 仍可以退回到启动抓取位姿。

#### 6.2.5 普通命名目标处理

- 如果 `configured_pose is None`
  - 说明该目标名不存在。
  - 会把所有可用目标名拼出来，方便定位配置问题。

这个函数说明：**吧台目标有特殊优先级逻辑，而普通目标没有。**

### 6.3 `_is_bar_target_name`（第 164–165 行）

这个函数只是一个集合判断：

- 输入目标名。
- 判断它是否属于吧台目标名集合。

## 7. 启动时吧台位姿抓取

### `_capture_startup_bar_pose`（第 167–202 行）

这个函数会尝试从 TF 中查询机器人当前位姿，并把它记成“启动时吧台位姿”。

流程如下：

1. 计算一个超时截止时间。
2. 循环尝试 `lookup_transform(global_goal_frame <- robot_base_frame)`。
3. 如果成功，就构造一个 `PoseStamped`。
4. 把平移和旋转复制进去。
5. 缓存到 `self.startup_bar_pose`。
6. 打印成功日志并返回 `True`。
7. 如果一直查不到 TF，直到超时则返回 `False`。

这个逻辑的本质是：**把机器人刚启动时的位置，视为“回吧台”的参考点。**

## 8. 导航执行逻辑

### 8.1 `_execute_move_base`（第 204–231 行）

这是 action 模式的执行函数。

#### 8.1.1 等待 action server

- `wait_for_server(wait_timeout)`
  - 先确认 `move_base` 是否存在。
  - 如果不存在，直接返回 `FAILURE_ACTION_UNAVAILABLE`。

#### 8.1.2 发送目标

- 构造 `MoveBaseGoal()`。
- 把 `goal_pose` 填到 `goal.target_pose`。
- `send_goal(goal)` 发送出去。

#### 8.1.3 等待结果

- `wait_for_result(rospy.Duration(self.timeout_sec))`
  - 在给定超时时间内等待导航完成。
- 如果超时：
  - 取消当前目标。
  - 返回 `FAILURE_TIMEOUT`。

#### 8.1.4 检查 action 状态码

- 如果状态是 `GoalStatus.SUCCEEDED`
  - 返回成功。
- 否则
  - 根据状态判断是“被拒绝类失败”还是“普通执行失败”。
  - 并把状态文本一起写入结果消息。

### 8.2 `_execute_topic_goal`（第 233–265 行）

这是不依赖 action server 的简化模式。

#### 8.2.1 发布目标

- `self.goal_publisher.publish(goal_pose)`
  - 把目标位姿直接发到 topic。

#### 8.2.2 轮询检查是否到达

- 计算一个截止时间 `deadline`。
- 用 `rospy.Rate(...)` 控制循环频率。
- 每轮：
  1. 先检查是否超时。
  2. 再调用 `_check_goal_reached(goal_pose)`。
  3. 若已到达，立即返回成功。
  4. 若未到达，则继续等待。

#### 8.2.3 适用场景

这种模式适合：

- 系统不能直接走 action。
- 但底层已有节点监听 `/move_base_simple/goal`。
- 且可以通过 TF 判断机器人是否到达。

## 9. 到达判定逻辑

### `_check_goal_reached`（第 267–288 行）

这个函数会实时比较“机器人当前位姿”和“目标位姿”。

#### 9.1 获取机器人当前位置

- `lookup_transform(goal_frame <- robot_base_frame)`
  - 这里查询的是：机器人当前在目标坐标系下的位置。

#### 9.2 计算位置误差

- `dx / dy`
  - 目标点与当前位置的平面差值。
- `dist = math.hypot(dx, dy)`
  - 平面距离误差。

#### 9.3 计算朝向误差

- 从机器人四元数中提取 `robot_yaw`。
- 从目标四元数中提取 `goal_yaw`。
- 用 `_normalize_angle` 计算规范化角度差。

#### 9.4 判定是否到达

- 只有同时满足：
  - `dist <= success_dist_thresh`
  - `yaw_error <= success_yaw_thresh`
  - 才认为真正到达。

这说明：这里的“到达”不仅看位置，还看朝向是否正确。

## 10. 命名目标加载逻辑

### 10.1 `_load_named_targets`（第 290–311 行）

这个函数负责从两类来源加载目标点：

1. 参数服务器。
2. YAML 文件。

加载顺序是：

- 先取参数服务器内容。
- 再用 YAML 文件内容更新。

这意味着：**后加载的 YAML 可能覆盖先加载的参数内容。**

之后会逐个目标执行：

- `_pose_from_config(cfg)`
  - 把配置字典转成 `PoseStamped`。
- 同时把别名 `aliases` 也映射到同一个位姿。

如果某个目标配置有问题，会跳过该目标并打 warning，而不是让整个节点启动失败。

### 10.2 `_read_targets_yaml`（第 313–316 行）

- 打开 YAML 文件。
- 用 `yaml.safe_load` 解析。
- 只读取 `restaurant.navigation_targets` 这一层。

### 10.3 `_pose_from_config`（第 318–338 行）

这个函数把单个目标配置转成 ROS 位姿对象。

- 要求 `cfg` 必须是字典。
- 读取：
  - `frame_id`
  - `position.x/y/z`
  - `orientation.x/y/z/w`
- 同时读取 `aliases` 别名列表。

### 10.4 `_copy_pose`（第 340–345 行）

这是一个简单的拷贝工具函数。

作用：

- 避免直接复用原对象，防止后续修改互相影响。
- 同时刷新时间戳为当前时间。

## 11. 目标坐标系预处理

### `_prepare_goal_pose`（第 347–394 行）

这个函数负责决定“目标点是否需要做 TF 坐标变换”。

#### 11.1 不做变换的情况

- 如果 `transform_goal_to_global_frame == False`
  - 直接保持原坐标系不变。
- 如果目标本来就在 `global_goal_frame`
  - 也无需变换。

#### 11.2 做变换的情况

- 如果目标坐标系不是全局坐标系，且启用了统一变换
  - 就调用 `self.tf_buffer.transform(...)` 转换。

#### 11.3 失败处理

- 若变换失败，则抛出 `RuntimeError`。
- 上层 `handle_nav_to_pose()` 会把它转换成 `FAILURE_TF_ERROR` 返回。

这个函数非常关键，因为它保证：

- 不同来源的目标点，最后都能落到统一的全局坐标系上执行。

## 12. `main()` 入口（第 397–404 行）

- 初始化 ROS 节点 `real_navigation_node`。
- 创建 `RealNavigationNode()` 对象，自动完成：
  - 参数读取
  - TF 初始化
  - 目标点加载
  - 服务注册
- `rospy.spin()` 保持节点常驻。

## 13. 这个节点的设计特点

### 13.1 统一了多种目标输入

它既支持：

- 通过名字导航到预设点。
- 也支持直接给任意动态位姿。

### 13.2 同时兼容 action 与 topic 两种导航模式

这让它可以适配不同底层导航系统：

- 标准 ROS 导航栈可走 `move_base` action。
- 简化版系统可走目标 topic + TF 判定。

### 13.3 对 TF 依赖较强

这个节点大量依赖 TF 用于：

- 启动抓取吧台位姿。
- 目标位姿坐标变换。
- 机器人是否到达目标的判定。

如果 TF 树不完整，它就容易失败。

### 13.4 到达判定更严格

很多系统只判断距离，这里同时判断：

- 平面距离。
- 朝向误差。

因此更适合需要精确停靠朝向的任务。

## 14. 一句话总结

`real_navigation_node.py` 是项目中的**真实导航服务节点**：它负责把上层“去某个位置”的请求，解析成真实导航目标，完成坐标变换、执行导航，并以统一结果返回给上层状态机。
