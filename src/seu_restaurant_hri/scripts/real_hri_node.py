#!/usr/bin/env python3
import rospy  # ROS Python 客户端库，用于注册服务和输出日志

from seu_restaurant_common.service_names import get_service_name  # 从统一配置中解析标准服务名
from seu_restaurant_hri.asr_bridge import LegacyASRBridge  # 把 HRI listen 接口桥接到底层 ASR 服务
from seu_restaurant_hri.tts_engine import speak  # 本地语音播报函数
from seu_restaurant_msgs.srv import ListenText, ListenTextResponse, SpeakText, SpeakTextResponse  # HRI 自定义服务类型


class RealHRINode:  # 真实 HRI 节点：对外统一提供说话与听写两个服务
    def __init__(self):
        self.asr_bridge = LegacyASRBridge()  # 初始化 ASR 桥接层，内部会去调用 speech_recognition_service
        speak_service_name = get_service_name("hri", "speak", private_param="~speak_service_name")  # 解析 speak 服务名
        listen_service_name = get_service_name("hri", "listen", private_param="~listen_service_name")  # 解析 listen 服务名
        self.speak_srv = rospy.Service(speak_service_name, SpeakText, self.handle_speak)  # 注册对外说话服务
        self.listen_srv = rospy.Service(listen_service_name, ListenText, self.handle_listen)  # 注册对外听写服务
        rospy.loginfo("[real_hri_node] Services ready: speak=%s listen=%s", speak_service_name, listen_service_name)  # 打印服务就绪信息方便确认

    def handle_speak(self, req):  # 处理上层发来的播报请求
        success, message = speak(req.text)  # 调用底层 TTS 引擎把文本说出来
        rospy.loginfo("[real_hri_node] SPEAK success=%s text=%s", success, req.text)  # 记录本次播报内容和执行结果
        return SpeakTextResponse(success=success, message=message)  # 把底层播报结果原样封装成 ROS 服务响应

    def handle_listen(self, req):  # 处理上层发来的听写请求
        success, text, message = self.asr_bridge.listen(prompt=req.prompt, timeout_sec=req.timeout_sec)  # 交给桥接层去调用真实 ASR 服务并整理结果
        rospy.loginfo(
            "[real_hri_node] LISTEN success=%s prompt='%s' timeout=%s -> '%s'",
            success,
            req.prompt,
            req.timeout_sec,
            text,
        )  # 输出本次听写请求的提示词、超时时间和识别结果
        return ListenTextResponse(success=success, text=text, message=message)  # 向上层返回识别到的文本与状态说明


def main():  # ROS 节点入口函数
    rospy.init_node("real_hri_node")  # 初始化真实 HRI 节点
    RealHRINode()  # 创建节点对象并完成 speak/listen 服务注册
    rospy.spin()  # 进入事件循环，持续等待上层模块调用 HRI 服务


if __name__ == "__main__":
    main()  # 直接运行脚本时从这里进入
