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
        self.msgMap={'vehicleElectronicState':'',
                     'carAbnormal':1}
        
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