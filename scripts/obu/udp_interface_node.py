#!/usr/bin/env python
# -*- coding:utf-8 -*- 
"""
获取底盘map数据, 序列化, 上传给服务器
获取服务器json数据,反序列化, 下发给底盘
"""
import unicodedata
import rospy
import json
import threading
import time
from obu.srv import OBUinterflowVehicle,OBUinterflowVehicleRequest,OBUinterflowVehicleResponse

import mySocket
from socket import MSG_DONTWAIT
from autoware_perception_msgs.msg import LampState,TrafficLightState,TrafficLightStateArray
from std_msgs.msg import Int16

import os,sys
# 导入上一级文件夹里的自定义模块
# sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
import config
import operator

class TcpInterfaceNode:
    def __init__(self):
        # 获取服务器和服务器端口
        self.addr=(config.server_ip, config.server_port)
        # 创建udp套接字
        self.udp_socket=mySocket.socket.socket(mySocket.socket.AF_INET,mySocket.socket.SOCK_DGRAM)
        self.udpSocket = mySocket.UdpSocket(self.addr, self.udp_socket) 

        rospy.Timer(rospy.Duration(1),self.callback_timer)

        # ros 客户端
        self.Registration_client = rospy.ServiceProxy("/obu/chassis/RegistratIonInformationReport",OBUinterflowVehicle)
        self.RealTimeInformation_client = rospy.ServiceProxy("/obu/chassis/RealTimeInformationUpload",OBUinterflowVehicle)
        self.StatusInformation_client = rospy.ServiceProxy("/obu/chassis/StatusInformationUpload",OBUinterflowVehicle)
        self.AbnormalInformation_client = rospy.ServiceProxy("/obu/chassis/AbnormalInformationUpload",OBUinterflowVehicle)
        self.Heartbeat_client = rospy.ServiceProxy("/obu/chassis/HeartbeatReport",OBUinterflowVehicle)

        self.Registration_client_srv = OBUinterflowVehicleRequest() 
        self.Registration_client_srv.requestCmd=0x60

        self.RealTimeInformation_client_srv = OBUinterflowVehicleRequest()
        self.RealTimeInformation_client_srv.requestCmd=0x61

        self.StatusInformation_client_srv = OBUinterflowVehicleRequest()
        self.StatusInformation_client_srv.requestCmd=0x62

        self.AbnormalInformation_client_srv  = OBUinterflowVehicleRequest()
        self.AbnormalInformation_client_srv.requestCmd=0x63

        self.Heartbeat_client_srv = OBUinterflowVehicleRequest()
        self.Heartbeat_client_srv.requestCmd=0x64

        self.TrafficLight_pub = rospy.Publisher("/perception/traffic_light_recognition/traffic_light_states",TrafficLightStateArray,queue_size=10,latch=True)
        self.TL_CountDown_pub= rospy.Publisher("/perception/traffic_light_recognition/traffic_light_count_down",Int16, queue_size=10)

        try:
            self.loop_recv_Thread = threading.Thread(target=self.loop_recv,args=())
        except:
            rospy.logerr ("Error: 创建线程失败！！！")

        # 循环接受服务器内容
        self.Status_last_resp = None
        self.Abnormal_last_resp = None
        rospy.loginfo(".......udp构造函数已完毕........")

    # --------------------------------------------------------------------------------------
    # 车载主机信息上行3.3.1.1-3.3.1.5
    # 创建rosservice 客户端
    # addr, udp_socket_obj
    # --------------------------------------------------------------------------------------

    # 3.3.1.1 车载消息注册
    def chassis_Registration_Information_report(self):
        # 1. 等到服务就绪
        self.Registration_client.wait_for_service()
        # 2. 请求信息获取响应信息
        time.sleep(1)
        resp = self.Registration_client.call(self.Registration_client_srv)
        # 3. 发送响应信息到服务器
        rospy.loginfo(".......发送OBU客户端注册........")
        self.udpSocket.single_send(self.Registration_client_srv.requestCmd, resp.json_str)
        # 4. 接受服务器信息
        recv_msg = self.udpSocket.single_recv()
        rospy.loginfo(".......OBU客户端注册成功........")

    # 3.3.1.2 车载主机车辆实时信息  --  单向 10hz          
    def chassis_Real_time_information_upload(self):
        # 1. 等到服务器就绪
        self.RealTimeInformation_client.wait_for_service()
        # 2. 请求信息 获取响应信息
        resp = self.RealTimeInformation_client.call(self.RealTimeInformation_client_srv)
        # 3. 发送响应信息到服务器
        self.udpSocket.single_send(self.RealTimeInformation_client_srv.requestCmd, resp.json_str)

    # 3.3.1.3 车载主机车辆状态信息  --  单向 触发           
    def chassis_Status_information_upload(self):
        # 1. 等到服务器就绪
        self.StatusInformation_client.wait_for_service()
        # 2. 请求信息 获取响应信息
        resp = self.StatusInformation_client.call(self.StatusInformation_client_srv)
        #判断两个json数据是否有变化
        if self.json_diff(resp,self.Status_last_resp):
            self.Status_last_resp = resp
            # 3. 发送响应信息到服务器
            self.udpSocket.single_send(self.StatusInformation_client_srv.requestCmd, json.dumps(resp, ensure_ascii=False).json_str)
    
    # 3.3.1.4 车载主机车辆异常信息  --  单向 触发           
    def chassis_Abnormal_information_upload(self):
        # 1. 等到服务器就绪
        self.AbnormalInformation_client.wait_for_service()
            # 2. 请求信息 获取响应信息
        resp = self.AbnormalInformation_client.call(self.AbnormalInformation_client_srv)
        if self.json_diff(resp,self.Abnormal_last_resp):
                self.Abnormal_last_resp = resp
                # 3. 发送响应信息到服务器
                self.udpSocket.single_send(self.AbnormalInformation_client_srv.requestCmd, resp.json_str)
        self.udpSocket.single_send(self.AbnormalInformation_client_srv.requestCmd, resp.json_str)   #test

    # 3.3.1.5 车载主机心跳上报     --  双向  1min          
    def chassis_heartbeat_report(self):
        self.udp_socket.sendto(mySocket.struct.pack("<BIB", 0x64, 0, 0), self.addr)

    
        #判断两个json是否相同
    def json_diff(self,json1,json2):
        #1、json数据转换成字典
        #if 是应为初始化的json2格式不同
        if (json2 != None):
            dict1 = json.loads(json1.json_str)
            dict2 = json.loads(json2.json_str)
        # 2、将两个字典按key排好序，然后使用zip()函数将两个字典对应的元素打包成元组。比较对应的元素的value是否相等。
            for src_list, dst_list in zip(sorted(dict1), sorted(dict2)):
                if dict1[src_list] != dict2[dst_list]:
                    return True
        else:
            return True
        
        return False


    # --------------------------------------------------------------------------------------
    # OBU 信息推送3.3.1.6-3.3.1.9
    # 创建rosservice 服务端
    # --------------------------------------------------------------------------------------

    def loop_recv(self):
        TrafficLight_count = 0
        while not rospy.is_shutdown():
            recv_msg = self.udpSocket.single_recv()
            if recv_msg[0]==0x70:
                TrafficLight_count += 1
                self.traffic_light_area_control_push(json.loads(recv_msg[2]),TrafficLight_count)

            elif recv_msg[0]==0x71:
                self.label_area_control_push(json.loads(recv_msg[2]))
            
            elif recv_msg[0]==0x80:
                self.reminder_push_in_alert_area(json.loads(recv_msg[2]))
            
            elif recv_msg[0]==0x84:
                self.speed_limit_and_green_wave_traffic_push(json.loads(recv_msg[2]))

            elif recv_msg[0]==0x64:
                rospy.logwarn("收到服务器心跳%d", recv_msg[0])
            
            else:
                rospy.logwarn("接收到的任务码不存在%d", recv_msg[0])
                rospy.logwarn( recv_msg)

#由于python2会将json传输的的数据解析为unicode，使用该函数将其转换为str
    def unicode_convert(self,input):
        if isinstance(input, dict):
            return {self.unicode_convert(key): self.unicode_convert(value) for key, value in input.iteritems()}
        elif isinstance(input, list):
            return [self.unicode_convert(element) for element in input]
        elif isinstance(input, unicode):#unicodedata
            return input.encode('utf-8')
        else:
            return input

    # 3.3.1.6 红绿灯区域控制推送   -- 单向 0.5s            
    def traffic_light_area_control_push(self, unicode_msg,count):
        dict_msg = self.unicode_convert(unicode_msg)#转换为str
        TrafficLight_msg = TrafficLightStateArray()
        TrafficLight_msg_id = TrafficLightState()
        TrafficLight_msg_Lamp = LampState()
        TL_CountDown = Int16()

        for i in range(len(dict_msg["lightDetails"])):
            TrafficLight_msg.header.frame_id = "map"
            TrafficLight_msg.header.stamp = rospy.Time.now()
            TrafficLight_msg.header.seq = count
            TrafficLight_msg_id.id = 125
            # TrafficLight_msg.states = TrafficLight_msg_id
            
            # TrafficLight_msg.states.lamp_states[i].confidence = dict_msg["lightDetails"][i]["timeLeft"]
            if dict_msg["lightDetails"][i]["if_current_light"] == 1 and dict_msg["lightDetails"][i]["type"] == 2:
                if dict_msg["lightDetails"][i]["state"] == 0:
                    TrafficLight_msg_Lamp.type = TrafficLight_msg_Lamp.RED
                    TrafficLight_msg_id.lamp_states.append(TrafficLight_msg_Lamp)
            
                elif dict_msg["lightDetails"][i]["state"] == 1:
                    TrafficLight_msg_Lamp.type = TrafficLight_msg_Lamp.YELLOW
                    TrafficLight_msg_id.lamp_states.append(TrafficLight_msg_Lamp)
                
                elif dict_msg["lightDetails"][i]["state"] == 2:
                    TrafficLight_msg_Lamp.type = TrafficLight_msg_Lamp.GREEN
                    TrafficLight_msg_id.lamp_states.append(TrafficLight_msg_Lamp)
                else:
                    TrafficLight_msg_Lamp.type = TrafficLight_msg_Lamp.UNKNOWN
                    TrafficLight_msg_id.lamp_states.append(TrafficLight_msg_Lamp)
                TL_CountDown.data = dict_msg["lightDetails"][i]["timeLeft"]

        TrafficLight_msg.states.append( TrafficLight_msg_id)
        self.TrafficLight_pub.publish(TrafficLight_msg)
        self.TL_CountDown_pub.publish(TL_CountDown)
                    


            # if_current_light = dict_msg["lightDetails"][i]["if_current_light"]
            # light_type = dict_msg["lightDetails"][i]["type"]
            # light_state = dict_msg["lightDetails"][i]["state"]
            # timeLeft = dict_msg["lightDetails"][i]["timeLeft"]
            # print("if_current_light:",if_current_light,"light_type:",light_type,
            # "light_state:",light_state,"timeLeft:",timeLeft)

    # 3.3.1.7 标牌区域域控制推送   -- 单向 3s              
    def label_area_control_push(self, unicode_msg):
        dict_msg =  self.unicode_convert(unicode_msg)#转换为str
        for i in range(len(dict_msg["signType"])):
            signType = dict_msg["signType"][i]
            print("signType:",signType)

    # 3.3.1.8 预警区域提醒推送     -- 单向 触发            
    def reminder_push_in_alert_area(self, unicode_msg):
        dict_msg =  self.unicode_convert(unicode_msg)#转换为str
        warningType = json.dumps(dict_msg["warningType"], ensure_ascii=False, encoding='utf-8')
        signType = dict_msg["signType"]
        evenType = dict_msg["evenType"]
        warningInfo = dict_msg["warningInfo"]
        warningSuggestion = dict_msg["warningSuggestion"]
        distant = dict_msg["distant"]
        print("warningType:",warningType,"signType:",signType,
        "evenType:",evenType,"warningInfo:",warningInfo,
        "warningSuggestion:",warningSuggestion,"distant:",distant)

    # 3.3.1.9 限速与绿波通行推送   -- 单向 1s              
    def speed_limit_and_green_wave_traffic_push(self, unicode_msg):
        dict_msg =  self.unicode_convert(unicode_msg)#转换为str
        speedLimit = dict_msg["speedLimit"]
        suggestSpeed = dict_msg["suggestSpeed"]
        print("speedLimit:",speedLimit,"suggestSpeed:",suggestSpeed)


    # 定时器-1s
    def callback_timer(self,event):
        # 关闭套接字
        if rospy.is_shutdown():
            self.udp_socket.close()
            rospy.loginfo("套接字关闭成功")
            



if __name__=="__main__":
    rospy.init_node("tcp_socket_node")
    rate = rospy.Rate(10)
    tcp_node = TcpInterfaceNode()
    rate.sleep()#延时初始化
    tcp_node.chassis_Registration_Information_report()
    one_min = 0# 计数器用于一分钟倒计时
    try :
        tcp_node.loop_recv_Thread.start()
    except:
        rospy.logerr ("Error: 启动线程失败！！！")

    while not rospy.is_shutdown():
        tcp_node.chassis_Real_time_information_upload()
        tcp_node.chassis_Abnormal_information_upload()
        tcp_node.chassis_Status_information_upload()
        if one_min == 600:
            tcp_node.chassis_heartbeat_report()
            one_min = 0
        rate.sleep()
        one_min += 1

        
    # 注册车辆信息
    rospy.spin()
    del tcp_node
    sys.exit()
    
