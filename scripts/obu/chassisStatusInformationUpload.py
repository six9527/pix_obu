#!/usr/bin/env python
# -*- coding:utf-8 -*- 
import rospy 
import json
import time
from obu.srv import OBUinterflowVehicle,OBUinterflowVehicleRequest,OBUinterflowVehicleResponse
# from pix_driver_msgs.msg import *
from pix_driver_msgs.msg import bms_report_512


class StatusInformationUpload:
    def __init__(self):
        self.server = rospy.Service("/obu/chassis/StatusInformationUpload",OBUinterflowVehicle, self.callback_doReq)
        self.sub_bms_report = rospy.Subscriber("/pix/bms_report",bms_report_512,self.bms_report_callback)
        self.count = 0

        self.msgMap={
            #剩余电量%
            "batteryQuantity":0,

            "oilQuantity":255,
            #理论可以计算得出（电池电压×电池容量×时速÷电机功率）
            "restdistance":-999,
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

            "cloudState":255,
            "_4gState":255,
            "_5gState":255,
            "ltevState":255,
            #摄像头状态
            "camerNum":4,

            # "cameraIp":"192.168.19.11",
            # "cameraConfig":None,
            "markType":1,
            "time":"0",
            "UTC":"0",
            "markNum":0
        }
        
    def bms_report_callback(self,msg):
        self.msgMap["batteryQuantity"] = msg.Battery_Soc
        self.msgMap["restdistance"] = msg.Battery_Soc*50
        self.msgMap["time"] = time.strftime('%Y-%m-%d %H:%M:%S',time.localtime())
        self.msgMap["UTC"] = str(time.time())
        self.msgMap["markNum"] = 0

    def callback_doReq(self, req):
        return json.dumps(self.msgMap,ensure_ascii=False)


if __name__ == "__main__":
    rospy.init_node("obu_chassis_StatusInformationUpload")
    StatusInformationUpload()
    rospy.spin()