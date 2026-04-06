# `speech_service.py` 中文注释说明

对应源码：`src/seu_speech_recognition/scripts/speech_service.py`

说明：本文件是**外部注释文档**，用于解释源码逻辑；**不修改原始 `.py` 文件**。

## 1. 文件职责

这个脚本实现了一个 ROS 服务节点 `speech_recognition_service`，核心职责是：

1. 从 ROS 参数服务器读取语音识别相关配置。
2. 初始化 `speech_recognition.Recognizer()` 识别器。
3. 按配置选择识别后端：`google` 或 `whisper`。
4. 在收到服务请求后，从麦克风采集一段音频。
5. 对音频做语音识别，并把结果通过 ROS 服务响应返回。

它本质上是“**麦克风采集 + ASR 后端封装 + ROS 服务接口**”三者的组合。

## 2. 总体执行流程

启动后，脚本的大致流程如下：

1. 先把 Conda 环境中的 `site-packages` 补进 `sys.path`。
2. 导入 `rospy`、`speech_recognition` 以及服务类型。
3. 创建 `SpeechRecognitionServiceNode` 对象。
4. 在构造函数里读取配置、初始化识别器、按需加载 Whisper 模型。
5. 创建 ROS 服务 `speech_recognition_service`。
6. 外部节点调用该服务时，节点开始录音、识别，并返回文本。

## 3. 逐段中文注释

### 3.1 文件头与环境修正（第 1–23 行）

- `#!/usr/bin/env python3`
  - 指定使用 Python3 解释器运行。
- `import os / sys / tempfile`
  - `os` 用来处理路径和文件删除。
  - `sys` 用来修改 Python 模块搜索路径。
  - `tempfile` 用来生成临时 wav 文件，供 Whisper 识别。
- `_append_conda_site_packages()`
  - 这是一个“环境兼容函数”。
  - 作用是把当前 Conda 环境，或者作者约定的 `~/miniconda3/envs/shibie/.../site-packages` 路径加入 `sys.path`。
  - 这样做的主要目的，是避免 ROS 默认 Python 环境里找不到 `speech_recognition`、`whisper`、`pyttsx3` 等第三方库。
- `_append_conda_site_packages()` 在导入 `rospy` 之前被调用
  - 这意味着后续导入三方库时，会优先从补充进去的环境中寻找依赖。

### 3.2 依赖导入（第 25–29 行）

- `import rospy`
  - ROS Python 节点的基础库。
- `import speech_recognition as sr`
  - 第三方语音识别库，负责调用麦克风并封装多种识别后端。
- `from seu_speech_recognition.srv import ...`
  - 导入项目自定义的 ROS 服务类型。
  - `seu_speech_recognition` 是服务类型本身。
  - `seu_speech_recognitionResponse` 用于构造服务返回值。

### 3.3 节点构造函数（第 31–71 行）

这里完成了节点的大多数初始化工作。

#### 3.3.1 读取 HRI 统一配置

- `real_cfg = rospy.get_param("/restaurant/hri/real", {})`
  - 从全局参数 `/restaurant/hri/real` 读取一整组真实 HRI 配置。
  - 这样做的好处是：语音识别节点与 HRI 节点共享一套配置来源，减少重复定义。

#### 3.3.2 读取服务与识别参数

- `self.service_name`
  - 服务名，默认是 `speech_recognition_service`。
- `self.backend`
  - 识别后端类型，支持 `google` 或 `whisper`。
- `self.language`
  - Whisper 的语言参数；为空时可由模型自动判断。
- `self.whisper_device`
  - Whisper 模型运行设备，如 `cpu` 或 `cuda`。
- `self.listen_timeout_sec`
  - 监听等待超时；超过这个时间仍未等到说话则报超时。
- `self.phrase_time_limit_sec`
  - 单次说话允许的最长持续时间。
- `self.ambient_duration_sec`
  - 环境噪声校准时长，用于在正式录音前估计噪声基线。
- `self.dynamic_energy_threshold`
  - 是否自动调节能量阈值。
- `self.energy_threshold`
  - 静态能量阈值，声音低于阈值可能被视为背景噪声。
- `self.pause_threshold`
  - 判断“说话是否结束”的停顿阈值。
- `self.non_speaking_duration`
  - 非语音片段相关的内部参数。
- `self.microphone_device_index`
  - 指定麦克风设备编号；`-1` 表示使用默认设备。
- `self.sample_rate`
  - 录音采样率。
- `self.use_task_as_initial_prompt`
  - 是否把服务请求里的 `task` 作为 Whisper 的 `initial_prompt`。

#### 3.3.3 设置模型路径

- `default_model_path = .../base.pt`
  - 默认认为脚本同目录下有 `base.pt` Whisper 模型文件。
- `self.whisper_model_path`
  - 可从 ROS 参数覆盖默认模型路径。

#### 3.3.4 初始化识别器对象

- `self.recognizer = sr.Recognizer()`
  - 创建 `speech_recognition` 识别器实例。
- 接着把若干阈值和行为参数写回 `Recognizer`
  - 这些参数会直接影响录音触发与静音判定行为。

#### 3.3.5 按需加载 Whisper 模型

- `if self.backend == "whisper": self._load_whisper_model()`
  - 如果当前选择的是 Whisper，则启动时就预加载模型。
  - 这样做的优点是：第一次服务请求时响应更快。
  - 代价是：启动阶段会更慢，且更占内存。

#### 3.3.6 注册 ROS 服务

- `self.service = rospy.Service(...)`
  - 创建 ROS 服务端。
  - 外部节点调用这个服务时，会转到 `self.handle_request` 执行。
- 最后打印一条 ready 日志
  - 方便确认当前服务名、后端类型、语言和模型来源。

### 3.4 服务主入口 `handle_request`（第 73–97 行）

这是整个节点最核心的入口函数。

#### 3.4.1 请求使能判断

- `if not req.enable:`
  - 如果调用方显式要求禁用本次识别，则直接返回失败。
  - 返回码 `10`，错误信息是 `request disabled`。

#### 3.4.2 采集音频

- `audio = self._capture_audio()`
  - 真正调用麦克风录音。
- 对异常做了细分：
  - `sr.WaitTimeoutError`
    - 表示在等待用户开口时超时。
  - `Exception`
    - 其他录音异常，比如麦克风不可用、权限问题、设备初始化失败等。

#### 3.4.3 执行语音识别

- `text = self._recognize(audio, req.task)`
  - 根据当前后端把音频转成文本。
- 对异常继续细分：
  - `sr.UnknownValueError`
    - 识别器拿到音频了，但无法理解内容。
  - `sr.RequestError`
    - 后端服务层面失败，例如 API 不可用。
  - `Exception`
    - 其他未预料异常。

#### 3.4.4 清洗结果并返回

- `text = (text or "").strip()`
  - 去掉前后空白，避免只返回换行或空格。
- 如果文本为空，则返回失败码 `33`。
- 否则记录日志并返回成功结果。

### 3.5 `_capture_audio`：从麦克风录音（第 99–115 行）

这个函数负责“真实采音”。

- `device_index = ...`
  - 如果配置的是非负数，就使用指定麦克风；否则用系统默认设备。
- `sample_rate = ...`
  - 如果采样率大于 0 就使用配置值，否则交给底层默认行为。
- `with sr.Microphone(...) as source:`
  - 打开麦克风设备，进入录音上下文。
- `adjust_for_ambient_noise(...)`
  - 先用一小段时间采集环境噪声，帮助动态调整灵敏度。
- `self.recognizer.listen(...)`
  - 阻塞等待用户说话并采集音频。
  - `timeout` 是“多久没等到说话就放弃”。
  - `phrase_time_limit` 是“说话最多录多久”。

### 3.6 `_recognize`：根据后端分发（第 117–122 行）

这是一个简单的后端选择器。

- 当 `backend == "google"`
  - 调 `recognize_google`，通常依赖联网接口。
- 当 `backend == "whisper"`
  - 调内部封装的 `_recognize_whisper`。
- 其他值
  - 直接抛出异常，提示后端不支持。

### 3.7 `_load_whisper_model`：加载 Whisper 模型（第 124–137 行）

- 先尝试 `import whisper`
  - 如果导入失败，直接抛出运行时异常。
- `model_spec = self.whisper_model_path`
  - 读取模型名或模型文件路径。
- `if os.path.exists(model_spec): model_spec = os.path.abspath(model_spec)`
  - 如果它是本地文件，就转成绝对路径，避免相对路径歧义。
- `whisper.load_model(model_spec, device=self.whisper_device)`
  - 真正把模型加载到指定设备上。

### 3.8 `_recognize_whisper`：Whisper 识别流程（第 139–160 行）

这个函数把录到的音频转换成 Whisper 可以处理的输入格式。

- 如果模型还没加载，则先补加载。
- `NamedTemporaryFile(..., suffix=".wav", delete=False)`
  - 创建一个临时 wav 文件。
  - 因为很多 Whisper 实现更方便直接读取文件路径。
- `temp_file.write(audio.get_wav_data())`
  - 把 `speech_recognition` 采集到的音频对象导出为 wav 二进制。
- `transcribe_kwargs = {"fp16": self.whisper_device != "cpu"}`
  - 如果不是 CPU，优先尝试半精度推理以节省显存、提升速度。
- 如果设置了语言，则传入 `language`。
- 如果启用了 `use_task_as_initial_prompt`，且本次请求带 `task_prompt`，则传给 Whisper。
  - 这有助于让模型在特定任务语境下更容易输出符合预期的文本。
- `result = self.whisper_model.transcribe(...)`
  - 真正执行转写。
- 最后在 `finally` 中删除临时文件
  - 防止 `/tmp` 或当前系统临时目录堆积大量中间文件。

### 3.9 `main()`：ROS 节点入口（第 163–170 行）

- `rospy.init_node("speech_recognition_service")`
  - 初始化 ROS 节点。
- `SpeechRecognitionServiceNode()`
  - 完成全部初始化。
- `rospy.spin()`
  - 进入 ROS 事件循环，等待服务请求。

## 4. 关键设计理解

### 4.1 为什么它叫“service”

因为这个脚本不是持续往外发布识别结果，而是等别的节点来“调用一次服务”，它才执行一次录音和识别。

### 4.2 为什么要区分 `google` 和 `whisper`

- `google`：通常接入简单，但可能依赖外部服务。
- `whisper`：本地离线能力更强，更适合机器人本地部署。

### 4.3 为什么要先录音再识别

因为这个节点不是流式 ASR，而是“单次请求式 ASR”：

1. 收到请求。
2. 录一段音。
3. 识别一次。
4. 返回文本。

## 5. 使用时的注意点

- 如果麦克风设备号不对，`_capture_audio()` 很可能失败。
- 如果 `whisper` 模型文件不存在，初始化阶段就会报错。
- 如果选择 `google` 后端，在无网络或网络受限环境下可能识别失败。
- `listen_timeout_sec` 太小会导致用户还没开口就超时。
- `phrase_time_limit_sec` 太小会把长句子截断。

## 6. 一句话总结

`speech_service.py` 是项目中的**真实语音识别服务节点**：它接到 ROS 服务请求后，从真实麦克风采音，再调用指定 ASR 后端把语音转成文本并返回。
