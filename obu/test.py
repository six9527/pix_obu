import rclpy
import time
import json
from rclpy.node import Node                    # ROS2 节点类
import threading
from std_srvs.srv import Trigger
from std_msgs.msg import Int16
from tier4_v2x_msgs.msg import VirtualTrafficLightState, VirtualTrafficLightStateArray
import os
current_file_path = os.path.abspath(__file__)
current_directory_path = os.path.dirname(current_file_path)
import sys
sys.path.append(current_directory_path)
import mySocket
import config

class ServiceClientNode(Node):
    def __init__(self,name):
        super().__init__(name)
        self.Registration_client = self.create_client(Trigger, 'RegistrationInformationReport')
        while not self.Registration_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Registration Service not available. Waiting...')

        self.RealTimeInformation_client = self.create_client(Trigger, 'RealTimeInformationUpload')
        while not self.RealTimeInformation_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('RealTimeInformation Service not available. Waiting...')

        self.StatusInformation_client = self.create_client(Trigger, 'StatusInformationUpload')
        while not self.StatusInformation_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('StatusInformation Service not available. Waiting...')

        self.AbnormalInformation__client = self.create_client(Trigger, 'AbnormalInformationUpload')
        while not self.AbnormalInformation__client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('AbnormalInformation Service not available. Waiting...')

        self.request = Trigger.Request()
        self.Registration_requestCmd = 0x60
        self.RealTimeInformation_requestCmd = 0x61
        self.StatusInformation_requestCmd = 0x62
        self.AbnormalInformation_requestCmd = 0x70#0x80#0x71#0x84#0x63
        self.Heartbeat_requestCmd=0x64

        self.servier_address = (config.server_ip,config.server_port)
        self.udpSocket = mySocket.UdpSocket(self.servier_address)

        #创建线程同步接收obu反馈消息
        try:
            self.Receive_thread = threading.Thread(target=self.Receive_thread,args=())
        except Exception as e:
            self.get_logger().info("Error: 创建线程失败！！！") 
    
        # 上一的异常信息和状态信息保存
        self.Status_last_resp = None
        self.Abnormal_last_resp = None

        self.warningType_pub = self.create_publisher(Int16, "/warning_type", 10)
        self.TL_CountDown_pub = self.create_publisher(Int16, "/TL_CountDown", 10)
        self.TrafficLight_pub = self.create_publisher(VirtualTrafficLightStateArray, "/traffic_light_state", 10)
    
    def Receive_thread(self):
        TrafficLight_count = 0
        while rclpy.ok():
            recv_msg = self.udpSocket.single_recv()
            if recv_msg[0]==0x70:
                TrafficLight_count += 1
                self.traffic_light_information(json.loads(recv_msg[2]),TrafficLight_count)

            elif recv_msg[0]==0x71:
                self.signage_information(json.loads(recv_msg[2]))
            
            elif recv_msg[0]==0x80:
                self.early_warning_information(json.loads(recv_msg[2]))
            
            elif recv_msg[0]==0x84:
                self.speed_limit_information(json.loads(recv_msg[2]))

            elif recv_msg[0]==0x64:
                self.get_logger().info("收到服务器心跳%d", recv_msg[0]) 
            else:
                # print(recv_msg[2])
                pass

    # 3.3.1.6 红绿灯区域控制推送   -- 单向 0.5s            
    def traffic_light_information(self, dict_msg,count):
        TrafficLight_msg_state = VirtualTrafficLightState()
        TrafficLight_msg = VirtualTrafficLightStateArray()
        TL_CountDown = Int16()
        if (len(dict_msg["id"])<=0):
            pass
        elif(dict_msg["id"] in config.traffic_light):
            # TrafficLight_msg.header.frame_id = "obu"
            TrafficLight_msg.stamp = self.get_clock().now().to_msg()
            # TrafficLight_msg.header.seq = count
            TrafficLight_msg_state.id = config.traffic_light[str(dict_msg["id"])]
            TrafficLight_msg_state.stamp = self.get_clock().now().to_msg()

            for i in range(len(dict_msg["lightDetails"])):
                if dict_msg["lightDetails"][i]["type"] == 2 and dict_msg["lightDetails"][i]["state"] != 3:
                    if dict_msg["lightDetails"][i]["state"] == 0:
                        TrafficLight_msg_state.type = "RED"
                    
                
                    elif dict_msg["lightDetails"][i]["state"] == 1:
                        TrafficLight_msg_state.type = "YELLOW"
                        
                    elif dict_msg["lightDetails"][i]["state"] == 2:
                        TrafficLight_msg_state.type = "GREEN"

                    else:
                        TrafficLight_msg_state.type = "UNKNOWN"
                        
                    TrafficLight_msg.states.append(TrafficLight_msg_state)
                    TL_CountDown.data = dict_msg["lightDetails"][i]["timeLeft"]

                    TrafficLight_msg.states.append(TrafficLight_msg_state)
                    self.TrafficLight_pub.publish(TrafficLight_msg)
                    self.TL_CountDown_pub.publish(TL_CountDown)
                elif dict_msg["lightDetails"][i]["type"] == 1 and dict_msg["lightDetails"][i]["state"] != 3:
                    if dict_msg["lightDetails"][i]["state"] == 0:
                        TrafficLight_msg_state.type = "RED"
                       
                
                    elif dict_msg["lightDetails"][i]["state"] == 1:
                        TrafficLight_msg_state.type = "YELLOW"
                        
                    
                    elif dict_msg["lightDetails"][i]["state"] == 2:
                        TrafficLight_msg_state.type = "GREEN"
                    
                    else:
                        TrafficLight_msg_state.type = "UNKNOWN"

                    TrafficLight_msg.states.append(TrafficLight_msg_state)
                    TL_CountDown.data = dict_msg["lightDetails"][i]["timeLeft"]

                    TrafficLight_msg_state.approval = True
                    TrafficLight_msg_state.is_finalized = True
                    TrafficLight_msg.states.append(TrafficLight_msg_state)
                    self.TrafficLight_pub.publish(TrafficLight_msg)
                    self.TL_CountDown_pub.publish(TL_CountDown)
        
    # 3.3.1.7 标牌区域域控制推送   -- 单向 3s              
    def signage_information(self, dict_msg):
        for i in range(len(dict_msg["signType"])):
            signType = dict_msg["signType"][i]
            print("signType:",signType)

    # 3.3.1.8 预警区域提醒推送     -- 单向 触发         
    def early_warning_information(self, dict_msg):
        warning_msg = Int16()
        warningType = dict_msg["warningType"]
        warning_msg.data= warningType
        signType = dict_msg["signType"]
        evenType = dict_msg["evenType"]
        warningInfo = dict_msg["warningInfo"]
        warningSuggestion = dict_msg["warningSuggestion"]
        distant = dict_msg["distant"]
        self.warningType_pub.publish(warning_msg)
        print("warningType:",warningType,"signType:",signType,
        "evenType:",evenType,"warningInfo:",warningInfo,
        "warningSuggestion:",warningSuggestion,"distant:",distant)
    
    # 3.3.1.9 限速与绿波通行推送   -- 单向 1s              
    def speed_limit_information(self, dict_msg):
        speedLimit = dict_msg["speedLimit"]
        suggestSpeed = dict_msg["suggestSpeed"]
        print("speedLimit:",speedLimit,"suggestSpeed:",suggestSpeed)

    # 向服务发送请求并将返回结果发送到OBU
    def send_request(self,requestCmd):
        if (requestCmd == self.Registration_requestCmd):
            future = self.Registration_client.call_async(self.request)

        elif (requestCmd == self.RealTimeInformation_requestCmd):
            future = self.RealTimeInformation_client.call_async(self.request)

        elif (requestCmd == self.StatusInformation_requestCmd):
            future = self.StatusInformation_client.call_async(self.request)

        elif (requestCmd == self.AbnormalInformation_requestCmd):
            future = self.AbnormalInformation__client.call_async(self.request)
            
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.udpSocket.single_send(requestCmd, str(response.message))
            else:
                self.get_logger().info('Service call failed: {}'.format(response.message))
        else:
            self.get_logger().error('Service call failed')

    # 向OBU发送心跳
    def heartbeat_report(self):
        self.udpSocket.single_send(0x60,"")

def main(args=None):
    rclpy.init(args=args)
    node = ServiceClientNode("service_client_node")
    one_min = 0# 计数器用于一分钟倒计时
    try :
        node.Receive_thread.start()
    except Exception as e:
        node.get_logger().error("Error: 启动线程失败！！！")
    while rclpy.ok():
        node.send_request(node.RealTimeInformation_requestCmd)
        node.send_request(node.AbnormalInformation_requestCmd)
        node.send_request(node.Registration_requestCmd)
        node.send_request(node.StatusInformation_requestCmd)
        if one_min >= 600:
            node.heartbeat_report()
            one_min = 0
        time.sleep(0.1)
        one_min +=1
    node.udpSocket.close()
    node.get_logger().info("套接字关闭成功")
    rclpy.shutdown()
