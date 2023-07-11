#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy 
import json
from std_srvs.srv import Trigger
from rclpy.node import Node                    # ROS2 节点类
from sensor_msgs.msg import NavSatFix, Imu
import tf2_geometry_msgs
import math
import os
current_file_path = os.path.abspath(__file__)
current_directory_path = os.path.dirname(current_file_path)
import sys
sys.path.append(current_directory_path)
import config


class RealTimeInformationUpload(Node):
    def __init__(self,name):
        super().__init__(name)   
        self.srv = self.create_service(Trigger,'/RealTimeInformationUpload', self.callback_doReq)    # 创建服务器对象（接口类型、服务名、服务器回调函数）
        self.sub_vcu_report = self.create_subscription(config.VcuReport, config.VcuTopic, self.vcu_report_callback, 10)
        self.sub_brake_report = self.create_subscription(config.BrakeReport, config.BrakeTopic, self.brake_report_callback, 10)
        self.sub_park_report = self.create_subscription(config.ParkReport, config.ParkTopic, self.park_report_callback, 10)
        self.sub_gear_report = self.create_subscription(config.GearReport, config.GearTopic, self.gear_report_callback, 10)
        self.sub_steering_report = self.create_subscription(config.SteeringReport, config.SteeringTopic, self.steering_report_callback, 10)
        self.sub_throttle_report = self.create_subscription(config.ThrottleReport, config.ThrottleTopic, self.throttle_report_callback, 10)

        self.sub_lon_lat = self.create_subscription(NavSatFix, '/fixposition/navsatfix', self.lon_lat_callback, 10)
        self.sub_imu = self.create_subscription(Imu, '/sensing/imu/imu_data', self.Imu_callback, 10)

        self.msgMap={
            'LocationType':31,
            #经纬度海拔
            'longitude':113.5951157,
            'latitude':22.74467066,
            'altitude':26.04,
            #定位精度
            'accuracy':1.0,

            'braking':0,
            'gear':78,
            'speed':0,
            'SWA':0,
            'turnLight':0,
            #车身位姿,imu数据deg
            'vehiclePosture':{
                'postureX':10,
                'postureY':0,
                'postureZ':0},

            #从车位置
            'relativePosition':{
                'speed2':45,
                'distanceR0':1,
                'distanceRY':3,
                'distanceRX':3,
                'distance0':3},

            'ifCarOnline':0,
            'throttle':0,
            #电机转速
            'rotationRate':50,

            #电机功率
            ' power':100,

            'carMode1':2,
            'carStatus':3,
            'SterringAngleSpd':0,

            # 车辆纵向，横向加速度
            'VehLatrlAccel':1,
            'VehLongAccell':1,

            #驾驶位门锁
            'DriverDoorLockSt':1,

            
            'BrakeLightSwitchSt':0,
            'ParkingLampSt':1,

            #住车灯，倒车灯
            'Reverse_light_status':0
        }

    def park_report_callback(self,msg):
        # self.msgMap['ParkingLampSt'] = msg.Parking_Actual
        self.msgMap['ParkingLampSt'] = msg.Parking_Actual


    def callback_doReq(self, request, response):
        response.message = json.dumps(self.msgMap, ensure_ascii=False)
        response.success = True
        return response

    def vcu_report_callback(self,msg):
        # self.msgMap['speed'] = msg.Vehicle_Speed
        self.msgMap['speed'] = msg.Vehicle_Speed
        self.msgMap['turnLight'] =msg.TurnLight_Actual

        #启动判断
        if ( msg.CarPower_State < 2):
            # 0不再线1在线
            self.msgMap['ifCarOnline'] = msg.CarPower_State
        else:
            self.msgMap['ifCarOnline'] = 0

        
        # 模式判断
        # 0x3: Standby Mode
        # 0x2: Emergency Mode
        # 0x1: Auto Mode
        # 0x0: Manual Remote Mode
        # msg.Vehicle_ModeState 
        #msg.vehicle_mode_state
        if (msg.Vehicle_ModeState == 0):#手动远程模式
            self.msgMap['carMode1'] = 1

        elif(msg.Vehicle_ModeState == 1):#自动驾驶模式
             self.msgMap['carMode1'] = 2

        elif(msg.Vehicle_ModeState == 2):#紧急模式
            self.msgMap['carMode1'] = 255

        elif(msg.Vehicle_ModeState == 3):#待机模式
            self.msgMap['carMode1'] = 255

        # 车辆状态
        # '0x7: crash
        # 0x6: error
        # 0x5: E-stop
        # 0x4: work
        # 0x3: 3
        # 0x2: 2
        # 0x1: 1
        # 0x0: init
        # '

        #     表示车辆状态，有未知
        # （FF），未启动(1)，停车
        # （2）或驾驶中(3)四种，数
        # 值类型如：1，未知为 255

        if (msg.CarWork_State == 4):#
            self.msgMap['carStatus'] = 3
        
        elif (msg.CarWork_State == 5):#
            self.msgMap['carStatus'] = 2
        else:
            self.msgMap['carStatus'] = 255
        
        # 车辆纵向，横向加速度
        self.msgMap['VehLongAccell'] = msg.Vehicle_Acc#车辆加速度
        #刹车灯
        self.msgMap['BrakeLightSwitchSt'] = msg.Brake_LightActual

    def brake_report_callback(self,msg):
        self.msgMap['braking'] = msg.Brake_PedalActual

    def gear_report_callback(self,msg):
        if (msg.Gear_Actual == 1):
            self.msgMap['gear'] = 80
        
        elif (msg.Gear_Actual == 2):
            self.msgMap['gear'] = 82
        
        elif (msg.Gear_Actual == 3):
            self.msgMap['gear'] = 78
            
        elif (msg.Gear_Actual == 4):
            self.msgMap['gear'] = 68
        else:
            self.msgMap['gear'] = 0xff

    def steering_report_callback(self,msg):
        self.msgMap['SWA'] = msg.Steer_AngleActual
        self.msgMap['SterringAngleSpd'] = msg.Steer_AngleActual 
        return 1

    def throttle_report_callback(self,msg):
        self.msgMap['throttle'] = msg.Dirve_ThrottlePedalActual
        return 1

    def lon_lat_callback(self, msg):
        self.msgMap['longitude'] = msg.longitude
        self.msgMap['latitude'] = msg.latitude
        self.msgMap['altitude'] = msg.altitude

    def Imu_callback(self,msg):
        euler = tf2_geometry_msgs.transformations.euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        self.msgMap['vehiclePosture']['postureX']= math.degrees(euler[1])
        self.msgMap['vehiclePosture']['postureY']= math.degrees(euler[0])
        self.msgMap['vehiclePosture']['postureZ']= math.degrees(euler[2])
        self.msgMap['VehLatrlAccel'] = msg.linear_acceleration.y

def main(args=None): 
    rclpy.init(args=args)                            # ROS2 Python接口初始化                         # ROS2 Python接口初始化
    node = RealTimeInformationUpload('RealTimeInformationUpload')       # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                                 # 循环等待ROS2退出
    node.destroy_node()                              # 销毁节点对象
    rclpy.shutdown()                                 # 关闭ROS2 Python接口