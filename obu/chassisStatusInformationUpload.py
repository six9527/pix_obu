#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy 
import json
import time
from std_srvs.srv import Trigger
from rclpy.node import Node                    # ROS2 节点类
import os
current_file_path = os.path.abspath(__file__)
current_directory_path = os.path.dirname(current_file_path)
import sys
sys.path.append(current_directory_path)
import config

class StatusInformationUpload(Node):
    def __init__(self,name):
        super().__init__(name) 
        self.srv = self.create_service(Trigger, "/StatusInformationUpload", self.callback_doReq)    # 创建服务器对象（接口类型、服务名、服务器回调函数）
        self.sub_vcu_report = self.create_subscription(config.BmsReport, config.BmsTopic, self.vcu_report_callback, 10)
        self.count = 0

        self.msgMap={
            #剩余电量%
            "batteryQuantity":0,

            "oilQuantity":255,
            #理论可以计算得出（电池电压×电池容量×时速÷电机功率）
            "restdistance":60,
            #定位状态1就绪,2未就绪,255未知
            "locationModelState":1,
            #电机状态1就绪,2未就绪,255未知
            "electricalMachineState":1,
            #传感器状态1就绪，2未就绪，255未知
            "sensorState":1,

            #无法获取不上报
            # "departure":" 广 州 市 南 沙 区 海 滨 路",
            # "dep_longitude":113.6145444,
            # "dep_latitude":22.74776556,

            "destination":"广州市",
            "des_longitude":113.28064,
            "des_latitude":23.125177,

            "cloudState":1,
            "_4gState":2,
            "_5gState":2,
            "ltevState":2,
            #摄像头状态
            "camerNum":4,

            "cameraIp":"192.168.19.11",
            "cameraConfig":1,
            "markType":1,
            "time":"0",
            "UTC":"0",
            "markNum":0
        }
        
    def vcu_report_callback(self,msg):
        self.msgMap["batteryQuantity"] = msg.Battery_Soc
        self.msgMap["restdistance"] = msg.Battery_Soc*60/100
        self.msgMap["time"] = time.strftime('%Y-%m-%d %H:%M:%S',time.localtime())
        self.msgMap["UTC"] = str(time.time())
        self.msgMap["markNum"] = 0

    def callback_doReq(self, request, response):
        response.message = json.dumps(self.msgMap, ensure_ascii=False)
        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)                            # ROS2 Python接口初始化                         # ROS2 Python接口初始化
    node = StatusInformationUpload('StatusInformationUpload')       # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                                 # 循环等待ROS2退出
    node.destroy_node()                              # 销毁节点对象
    rclpy.shutdown()                                 # 关闭ROS2 Python接口 