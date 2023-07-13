#!/usr/bin/env python
# -*- coding:utf-8 -*- 
"""
3.3.1.4 车载主机车辆异常信息     -- 触发           chassis abnormal Information report
 
"""
    
import rclpy                                     # ROS2 Python接口库
from rclpy.node import Node                    # ROS2 节点类
import json
from std_srvs.srv import Trigger

class AbnormalInformationUpload(Node):
    def __init__(self,name):
        super().__init__(name)                                                             # ROS2节点父类初始化
        self.srv = self.create_service(Trigger, '/AbnormalInformationUpload', self.callback_doReq)    # 创建服务器对象（接口类型、服务名、服务器回调函数）
        # self.msgMap={'vehicleElectronicState':'',
        #              'carAbnormal':1}
        # self.msgMap={"speedLimit":0,
        #              "suggestSpeed":30}
        # self.msgMap = {"signType":[11,38,37]}
        # self.msgMap = {"warningType":1,"signType":0,"evenType":0,
        #                "warningInfo":"前 车 碰 撞 预 警",
        #                "warningSuggestion":"请减速，前方有碰撞危险","distant":40}
        self.msgMap = {"lightDetails":
                       [{"if_current_light":1,"type":0,"state":0,"timeLeft":20},
                        {"if_current_light":1,"type":1,"state":2,"timeLeft":30},
                        {"if_current_light":0,"type":3,"state":2,"timeLeft":30},
                        {"if_current_light":0,"type":3,"state":2,"timeLeft":4}],
                        "id":"light219218"}
        
        
    def callback_doReq(self, request, response):
        response.message = json.dumps(self.msgMap)
        response.success = True
        return response
    
def main(args=None): 
    rclpy.init(args=args)                            # ROS2 Python接口初始化
    node = AbnormalInformationUpload("AbnormalInformationUpload")       # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                                 # 循环等待ROS2退出
    node.destroy_node()                              # 销毁节点对象
    rclpy.shutdown()                                 # 关闭ROS2 Python接口