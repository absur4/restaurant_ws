#!/usr/bin/env python3
import os  # 处理路径、文件存在性判断与临时文件删除
import sys  # 访问 Python 解释器版本与模块搜索路径
import tempfile  # 为 Whisper 转写创建临时 wav 文件


def _append_conda_site_packages():  # 将 Conda 环境的 site-packages 注入 sys.path，避免 ROS 环境找不到第三方依赖
    version_tag = "python{}.{}".format(sys.version_info[0], sys.version_info[1])  # 生成当前 Python 版本目录名，例如 python3.8
    candidates = []  # 收集可能存在的 Conda site-packages 路径

    conda_prefix = os.environ.get("CONDA_PREFIX", "").strip()  # 优先读取当前激活 Conda 环境的前缀路径
    if conda_prefix:
        candidates.append(os.path.join(conda_prefix, "lib", version_tag, "site-packages"))  # 拼出当前 Conda 环境的三方库目录

    home_dir = os.path.expanduser("~")  # 获取用户主目录
    candidates.append(os.path.join(home_dir, "miniconda3", "envs", "shibie", "lib", version_tag, "site-packages"))  # 兼容作者固定使用的 shibie 环境

    for candidate in candidates:
        if os.path.isdir(candidate) and candidate not in sys.path:
            sys.path.insert(0, candidate)  # 放到最前面，确保优先从这些环境中导入依赖


_append_conda_site_packages()  # 在导入第三方库前先补齐依赖路径

import rospy  # ROS Python 客户端库
import speech_recognition as sr  # 语音采集与语音识别统一封装库

from seu_speech_recognition.srv import seu_speech_recognition, seu_speech_recognitionResponse  # 自定义 ASR ROS 服务类型


class SpeechRecognitionServiceNode:  # 真实语音识别服务节点：负责录音、转写并通过 ROS 服务返回结果
    def __init__(self):
        real_cfg = rospy.get_param("/restaurant/hri/real", {})  # 从统一 HRI 配置中读取真实语音识别相关参数
        self.service_name = rospy.get_param("~service_name", real_cfg.get("legacy_asr_service_name", "speech_recognition_service"))  # 服务名，供 HRI 桥接层调用
        self.backend = str(rospy.get_param("~backend", real_cfg.get("asr_backend", "whisper"))).strip().lower()  # 当前识别后端，支持 whisper 或 google
        self.language = str(rospy.get_param("~language", real_cfg.get("asr_language", ""))).strip() or None  # Whisper 语言提示，为空时让模型自动判断
        self.whisper_device = str(rospy.get_param("~whisper_device", real_cfg.get("whisper_device", "cpu"))).strip() or "cpu"  # Whisper 推理设备，如 cpu/cuda
        self.listen_timeout_sec = float(rospy.get_param("~listen_timeout_sec", real_cfg.get("asr_listen_timeout_sec", 8.0)))  # 等待用户开始说话的最长时间
        self.phrase_time_limit_sec = float(rospy.get_param("~phrase_time_limit_sec", real_cfg.get("asr_phrase_time_limit_sec", 8.0)))  # 单次语音片段允许的最长录制时间
        self.ambient_duration_sec = float(rospy.get_param("~ambient_duration_sec", real_cfg.get("asr_ambient_duration_sec", 0.5)))  # 环境噪声校准时长
        self.dynamic_energy_threshold = bool(rospy.get_param("~dynamic_energy_threshold", real_cfg.get("dynamic_energy_threshold", True)))  # 是否让库自动调整能量阈值
        self.energy_threshold = int(rospy.get_param("~energy_threshold", real_cfg.get("energy_threshold", 300)))  # 静态能量阈值，影响起始语音检测灵敏度
        self.pause_threshold = float(rospy.get_param("~pause_threshold", real_cfg.get("pause_threshold", 2)))  # 停顿多久判定一句话结束
        self.non_speaking_duration = float(rospy.get_param("~non_speaking_duration", real_cfg.get("non_speaking_duration", 0.5)))  # 非语音片段时长参数，影响静音切分
        self.microphone_device_index = int(rospy.get_param("~device_index", real_cfg.get("asr_device_index", -1)))  # 麦克风设备编号，-1 表示使用默认设备
        self.sample_rate = int(rospy.get_param("~sample_rate", real_cfg.get("asr_sample_rate", 16000)))  # 麦克风采样率
        self.use_task_as_initial_prompt = bool(rospy.get_param("~use_task_as_initial_prompt", real_cfg.get("use_task_as_initial_prompt", False)))  # 是否把任务提示词传给 Whisper 作为先验提示

        default_model_path = os.path.join(os.path.dirname(__file__), "base.pt")  # 默认模型文件与脚本同目录存放
        self.whisper_model_path = str(rospy.get_param("~whisper_model_path", real_cfg.get("whisper_model_path", default_model_path))).strip()  # Whisper 模型路径或模型名
        self.google_language = str(rospy.get_param("~google_language", real_cfg.get("google_language", "en-US"))).strip() or "en-US"  # Google 识别后端的语言参数

        self.recognizer = sr.Recognizer()  # 创建 speech_recognition 识别器对象
        self.recognizer.dynamic_energy_threshold = self.dynamic_energy_threshold  # 设置动态能量阈值开关
        self.recognizer.energy_threshold = self.energy_threshold  # 设置固定能量阈值
        self.recognizer.pause_threshold = self.pause_threshold  # 设置停顿判定阈值
        self.recognizer.non_speaking_duration = self.non_speaking_duration  # 设置非语音时长参数

        self.whisper_model = None  # Whisper 模型对象，延迟或预加载后赋值
        self.whisper_module = None  # Whisper Python 模块引用，便于调试和后续扩展
        if self.backend == "whisper":
            self._load_whisper_model()  # 若当前配置使用 Whisper，则在启动时预先加载模型减少首轮时延

        self.service = rospy.Service(self.service_name, seu_speech_recognition, self.handle_request)  # 注册 ROS 服务，收到请求后回调 handle_request
        rospy.loginfo(
            "[speech_recognition_service] ready service=%s backend=%s language=%s model=%s",
            self.service_name,
            self.backend,
            self.language or "auto",
            self.whisper_model_path if self.backend == "whisper" else self.google_language,
        )  # 打印启动信息，便于确认服务名、后端、语言和模型配置

    def handle_request(self, req):  # ROS 服务主入口：一次调用对应一次录音和识别过程
        if not req.enable:
            return seu_speech_recognitionResponse(False, 10, "", "request disabled")  # 调用方显式禁用本次请求时直接返回

        try:
            audio = self._capture_audio()  # 从真实麦克风阻塞录制一段语音
        except sr.WaitTimeoutError:
            return seu_speech_recognitionResponse(False, 20, "", "listen timeout waiting for speech")  # 长时间没等到用户开口
        except Exception as exc:
            return seu_speech_recognitionResponse(False, 21, "", "audio capture failed: {}".format(exc))  # 麦克风不可用或录音阶段异常

        try:
            text = self._recognize(audio, req.task)  # 调用具体后端把录音转成文本
        except sr.UnknownValueError:
            return seu_speech_recognitionResponse(False, 30, "", "speech not understood")  # 录到了音，但无法识别出可理解文本
        except sr.RequestError as exc:
            return seu_speech_recognitionResponse(False, 31, "", "ASR request failed: {}".format(exc))  # 后端请求层面失败
        except Exception as exc:
            return seu_speech_recognitionResponse(False, 32, "", "ASR exception: {}".format(exc))  # 识别过程中的其他未分类异常

        text = (text or "").strip()  # 去掉前后空白，防止只返回空格或换行
        if not text:
            return seu_speech_recognitionResponse(False, 33, "", "ASR returned empty text")  # 后端返回空字符串也视为失败
        rospy.loginfo("[speech_recognition_service] recognized: %s", text)  # 在日志中打印最终识别文本方便调试
        return seu_speech_recognitionResponse(True, 0, text, "ok")  # 成功时把文本塞进 word 字段返回

    def _capture_audio(self):  # 从麦克风采集原始语音音频
        device_index = self.microphone_device_index if self.microphone_device_index >= 0 else None  # 负数代表不指定设备号，交给系统选择默认设备
        sample_rate = self.sample_rate if self.sample_rate > 0 else None  # 若采样率非法则交给底层使用默认值
        with sr.Microphone(device_index=device_index, sample_rate=sample_rate) as source:  # 打开麦克风输入流
            if self.ambient_duration_sec > 0.0:
                self.recognizer.adjust_for_ambient_noise(source, duration=self.ambient_duration_sec)  # 先根据环境噪声自动校准识别灵敏度
            rospy.loginfo(
                "[speech_recognition_service] listening timeout=%.1fs phrase_time_limit=%.1fs device_index=%s",
                self.listen_timeout_sec,
                self.phrase_time_limit_sec,
                str(device_index),
            )  # 输出本轮录音的等待时长、单句最大时长与设备号
            return self.recognizer.listen(
                source,
                timeout=max(0.1, float(self.listen_timeout_sec)),
                phrase_time_limit=max(0.1, float(self.phrase_time_limit_sec)),
            )  # 阻塞监听，直到录到一段完整语音或触发超时

    def _recognize(self, audio, task_prompt):  # 根据配置把音频分发给不同 ASR 后端
        if self.backend == "google":
            return self.recognizer.recognize_google(audio, language=self.google_language)  # 使用 Google Web Speech 后端识别
        if self.backend == "whisper":
            return self._recognize_whisper(audio, task_prompt)  # 使用本地 Whisper 模型识别
        raise RuntimeError("unsupported ASR backend '{}'".format(self.backend))  # 避免配置拼错后静默失败

    def _load_whisper_model(self):  # 加载 Whisper 模型到内存中
        try:
            import whisper  # 延迟导入，避免在不使用 Whisper 时增加依赖负担
        except Exception as exc:
            raise RuntimeError("failed to import whisper backend: {}".format(exc))  # 缺少库时尽早报错，便于定位环境问题

        model_spec = self.whisper_model_path  # 允许配置为模型名，也允许配置为本地模型文件路径
        if not model_spec:
            model_spec = "base"  # 未提供时退回 Whisper 默认 base 模型
        if os.path.exists(model_spec):
            model_spec = os.path.abspath(model_spec)  # 本地文件存在时转成绝对路径，避免相对路径歧义

        self.whisper_module = whisper  # 记录模块引用，主要便于调试
        self.whisper_model = whisper.load_model(model_spec, device=self.whisper_device)  # 把模型加载到指定推理设备

    def _recognize_whisper(self, audio, task_prompt):  # Whisper 后端的具体转写实现
        if self.whisper_model is None:
            self._load_whisper_model()  # 若尚未加载模型，则在首次调用时懒加载

        with tempfile.NamedTemporaryFile(prefix="speech_recognition_", suffix=".wav", delete=False) as temp_file:  # 先把音频对象落成临时 wav 文件
            temp_file.write(audio.get_wav_data())  # 导出成 Whisper 常用的 wav 文件格式
            temp_path = temp_file.name  # 保存临时文件路径供后续转写使用
        try:
            transcribe_kwargs = {"fp16": self.whisper_device != "cpu"}  # GPU 模式下优先启用半精度，CPU 模式禁用 fp16
            if self.language:
                transcribe_kwargs["language"] = self.language  # 显式指定语言可以减少误判
            if self.use_task_as_initial_prompt and task_prompt:
                transcribe_kwargs["initial_prompt"] = str(task_prompt)  # 用任务提示词引导模型更贴合当前语境
            result = self.whisper_model.transcribe(temp_path, **transcribe_kwargs)  # 执行 Whisper 转写
            if isinstance(result, dict):
                return result.get("text", "")  # OpenAI Whisper 常见返回结构里，文本位于 text 字段
            return str(result)  # 兼容其他实现直接返回字符串或自定义对象
        finally:
            try:
                os.remove(temp_path)  # 无论识别成功与否，都清理中间临时文件
            except OSError:
                pass  # 若删除失败则忽略，避免影响主流程返回


def main():  # ROS 节点入口函数
    rospy.init_node("speech_recognition_service")  # 初始化节点名，与 launch 中的节点实例相对应
    SpeechRecognitionServiceNode()  # 构造节点对象并注册服务
    rospy.spin()  # 进入事件循环，持续等待外部服务请求


if __name__ == "__main__":
    main()  # 以脚本方式直接运行时，从这里进入主逻辑
