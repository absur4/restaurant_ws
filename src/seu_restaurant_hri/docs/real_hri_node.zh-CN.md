# `real_hri_node.py` 中文注释说明

对应源码：`src/seu_restaurant_hri/scripts/real_hri_node.py`

说明：本文件是**外部注释文档**，只做中文解释，**不修改原始源码**。

## 1. 文件职责

这个脚本实现了“真实 HRI 节点”，主要提供两个 ROS 服务：

1. `speak`：让机器人说话。
2. `listen`：让机器人听用户说话并返回文本。

它本身并不直接做完整的 ASR 和 TTS 细节，而是起一个**统一入口和封装协调器**的作用：

- 语音播报交给 `tts_engine.speak()`。
- 语音识别交给 `LegacyASRBridge`。

## 2. 总体执行流程

启动后，这个节点的行为很简单：

1. 初始化 ROS 节点。
2. 创建 `LegacyASRBridge` 对象，作为听写能力的桥接层。
3. 注册 `speak` 与 `listen` 两个服务。
4. 其他模块通过统一的 HRI 服务名来调用它。
5. 它再把请求分发给 TTS 或 ASR 子模块。

## 3. 逐段中文注释

### 3.1 依赖导入（第 1–7 行）

- `import rospy`
  - ROS Python 节点基础库。
- `get_service_name`
  - 用于从统一配置里取得 HRI 服务名，避免把服务名写死在代码里。
- `LegacyASRBridge`
  - 语音识别桥接器。
  - 这个类负责调用底层语音识别服务，并把返回值整理成 HRI 节点可用的格式。
- `speak`
  - 来自 `tts_engine` 的播报函数。
- `ListenText / SpeakText`
  - 项目自定义的 HRI 服务类型。

### 3.2 `RealHRINode.__init__`（第 10–17 行）

构造函数完成两个关键动作：

#### 3.2.1 初始化 ASR 桥接器

- `self.asr_bridge = LegacyASRBridge()`
  - 这里创建了一个“听”的能力代理。
  - 以后凡是收到 `listen` 请求，都会转交给这个桥接器处理。

#### 3.2.2 获取并注册服务名

- `speak_service_name = get_service_name("hri", "speak", ...)`
  - 读取 speak 服务名，默认通常是 `/restaurant/hri/speak`。
- `listen_service_name = get_service_name("hri", "listen", ...)`
  - 读取 listen 服务名，默认通常是 `/restaurant/hri/listen`。
- `rospy.Service(..., self.handle_speak)`
  - 注册说话服务。
- `rospy.Service(..., self.handle_listen)`
  - 注册听写服务。
- 最后的日志用于确认服务是否成功挂起。

这一段的核心意思是：**RealHRINode 对外暴露统一的 HRI 接口。**

### 3.3 `handle_speak`：处理播报请求（第 19–22 行）

这是“机器人说话”服务的实现。

- `success, message = speak(req.text)`
  - 直接把请求中的文本 `req.text` 交给 TTS 引擎处理。
- `rospy.loginfo(...)`
  - 记录日志，便于调试当前说了什么，以及是否成功。
- `return SpeakTextResponse(...)`
  - 把 TTS 执行结果包装成 ROS 服务响应返回。

这个函数很薄，说明设计者故意把 TTS 逻辑抽到 `tts_engine.py` 中，避免 HRI 节点本身变得过于复杂。

### 3.4 `handle_listen`：处理听写请求（第 24–33 行）

这是“机器人听用户说话”服务的实现。

- `self.asr_bridge.listen(prompt=req.prompt, timeout_sec=req.timeout_sec)`
  - 把请求中的提示词和超时参数，传递给 ASR 桥接层。
- 返回三个值：
  - `success`：这次识别是否成功。
  - `text`：识别到的文本内容。
  - `message`：附加说明，比如错误原因或后端说明。
- 日志里会打印：
  - 是否成功。
  - 本次 prompt 是什么。
  - 超时时间是多少。
  - 最终识别到了什么文本。
- 最后返回 `ListenTextResponse`。

这个函数说明：**real_hri_node 不自己直接录音识别，而是通过桥接器访问底层 ASR 服务。**

### 3.5 `main()`：程序入口（第 36–43 行）

- `rospy.init_node("real_hri_node")`
  - 初始化 ROS 节点。
- `RealHRINode()`
  - 创建对象并自动完成服务注册。
- `rospy.spin()`
  - 保持节点常驻，等待服务调用。

## 4. 它在整体系统中的位置

可以把 `real_hri_node.py` 理解成 HRI 子系统的“门面层”：

- 上层状态机、任务控制逻辑，只需要调用统一的 HRI 服务。
- 下层真实语音识别服务、TTS 引擎细节，被屏蔽在内部。

这样设计有几个好处：

1. 上层代码不需要关心底层 ASR 是 Whisper 还是 Google。
2. 以后如果更换 TTS/ASR 实现，只需要改桥接层或引擎层。
3. mock HRI 和 real HRI 可以保持相同的服务接口，方便切换。

## 5. 与 mock_hri_node 的区别

- `mock_hri_node.py`
  - 直接返回预设文本，不接真实麦克风，也不做真实播报。
- `real_hri_node.py`
  - 真正调用底层语音识别服务和 TTS 引擎。

所以它是“**真实模式下的 HRI 服务封装层**”。

## 6. 关键设计理解

### 6.1 为什么要引入 `LegacyASRBridge`

因为项目里底层语音识别服务与 HRI 层之间存在一层适配需求：

- HRI 希望暴露统一的 `listen` 接口。
- 但底层 ASR 服务可能使用的是历史遗留服务类型和字段名。

桥接层可以把两边对接起来，而不让 `real_hri_node.py` 里充满兼容代码。

### 6.2 为什么 `handle_speak` 和 `handle_listen` 这么薄

这是典型的分层设计：

- 节点层负责 ROS 通信。
- 业务层负责听和说的真正实现。

这样便于测试、维护和替换实现。

## 7. 一句话总结

`real_hri_node.py` 是系统中的**真实人机交互服务入口**：对外提供统一的“说”和“听”接口，对内把播报交给 TTS，把听写交给 ASR 桥接器。
