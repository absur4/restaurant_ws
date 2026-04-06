# Speech_recognition Service Package

## Overview

这是一个ROS功能包，用于实现语音识别服务的封装，自定义服务消息seu_speech_recognition.srv，编写服务脚本speech_service.py和speech_recognition_api.py。speech_test.py用于测试语音识别服务是否正常

## Basic Usage

要启动该服务，运行如下命令：

    rosrun seu_speech_recognition speech_service.py
    rosrun seu_speech_recognition real_time_speech_topic.py

要在终端中测试该服务的调用，请运行：

    rosservice call /speech_recognition_service false command

## Node: speech_recognition_node

这是一个语音识别服务节点，它通过麦克风采集用户语音，录音完毕后将音频保存到本地，识别时调用百度api对本地音频进行识别，返回识别结果及错误码。
路径修改：speech_service.py 和 speech_recognition_api.py 都需要修改需识别音频的全局路径。
核心代码分为两个模块
1.录音模块
目的是获取音频上传给百度进行识别,其中adjust_for_ambient_noise()函数和listen()函数可以根据需要调整参数
2.识别模块
把刚刚录制好的音频转换成文字,这里选用了百度API,改动access_token后可以直接使用.

### Services

* **`speech_recognition_service`** ([seu_speech_recognition_srv/bool])

    随意输入true或false均可启动服务
