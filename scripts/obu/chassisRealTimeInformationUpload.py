#!/usr/bin/env python
# -*- coding:utf-8 -*- 
import rospy 
import json
from obu.srv import *
from pix_driver_msgs.msg import BrakeReport
from pix_driver_msgs.msg import GearReport
from pix_driver_msgs.msg import SteerReport
from pix_driver_msgs.msg import VcuReport
from pix_driver_msgs.msg import ParkReport
from pix_driver_msgs.msg import ThrottleReport
from sensor_msgs.msg import NavSatFix, Imu
from pix_driver_msgs.msg import VehicleModeCommand
from tf import transformations
from std_msgs.msg import Float32


class RealTimeInformationUpload:
    def __init__(self):
        self.server = rospy.Service("/obu/chassis/RealTimeInformationUpload",OBUinterflowVehicle, self.callback_doReq)

                #转向，车速，启动状态，驾驶模式，车辆状态，加速度，刹车灯
        self.sub_vcu_report = rospy.Subscriber('/pix/vcu_report', VcuReport, self.vcu_report_callback)
        #刹车
        self.sub_brake_report = rospy.Subscriber("/pix/brake_report",BrakeReport,self.brake_report_callback)
        #倒车
        self.sub_park_report = rospy.Subscriber("/pix/park_report",ParkReport,self.park_report_callback)
        #档位状态
        self.sub_gear_report = rospy.Subscriber("/pix/gear_report",GearReport,self.gear_report_callback)
        #方向盘转角，转角速度
        self.sub_steering_report = rospy.Subscriber("/pix/steering_report",SteerReport,self.steering_report_callback)
        #油门状态
        self.sub_throttle_report = rospy.Subscriber("/pix/throttle_report",ThrottleReport,self.throttle_report_callback)
        #车门
        self.vehicle_command = rospy.Subscriber("/pix/vehicle_command",VehicleModeCommand,self.vehicle_command_callback)
        # ？？？？
        # 经纬度，海拔，车身姿态，定位状态，从车速度和距离，电机功率转速，横向纵向加速度，住车灯，刹车灯，倒车灯？？
        self.sub_lon_lat = rospy.Subscriber("/fix", NavSatFix, self.lon_lat_callback)#经纬度
        # /sensing/imu/imu_data
        self.sub_imu= rospy.Subscriber("/chc_imu", Imu, self.Imu_callback)#imu数据
        self.sub_heading= rospy.Subscriber("/chc/heading", Float32, self.heading_callback)#imu数据


        self.msgMap={
            "LocationType":31,
            "heading":0,
            #经纬度海拔
            "longitude":113.5951157,
            "latitude":22.74467066,
            "altitude":26.04,
            #定位精度
            "accuracy":1.0,

            "braking":0,
            "gear":bytes.decode("N"),
            "speed":0,
            "SWA":0,
            "turnLight":0,
            #车身位姿，imu数据deg
            "vehiclePosture":{
                "postureX":0,
                "postureY":0,
                "postureZ":0},

            #从车位置
            "relativePosition":{
                "speed2":45,
                "distanceR0":1,
                "distanceRY":3,
                "distanceRX":3,
                "distance0":3},

            "ifCarOnline":1,
            "throttle":30,
            #电机转速
            "rotationRate":50,

            #电机功率
            " power":100,

            "carMode1":2,
            "carStatus":3,
            "SterringAngleSpd":34,

            # 车辆纵向，横向加速度
            "VehLatrlAccel":1,
            "VehLongAccell":1,

            #驾驶位门锁
            "DriverDoorLockSt":0,

            
            "BrakeLightSwitchSt":0,
            "ParkingLampSt":1,

            #住车灯，倒车灯
            "Reverse_light_status":0
        }

    def park_report_callback(self,msg):
        self.msgMap["ParkingLampSt"] = msg.parking_actual

    def heading_callback(self,msg):
        self.msgMap["heading"] = msg.data


    def callback_doReq(self, req):
        # print(self.msgMap)
        return json.dumps(self.msgMap,ensure_ascii=False)

    def vcu_report_callback(self,msg):
        self.msgMap["speed"] = msg.speed
        self.msgMap["turnLight"] =msg.turn_light_actual

        #启动判断
        # if ( msg.CarPower_State < 2):
        #     # 0不再线1在线
        #     self.msgMap["ifCarOnline"] = msg.CarPower_State
        # else:
        #     self.msgMap["ifCarOnline"] = 0

        
        # 模式判断
        # 0x3: Standby Mode
        # 0x2: Emergency Mode
        # 0x1: Auto Mode
        # 0x0: Manual Remote Mode
        # msg.Vehicle_ModeState 
        #msg.vehicle_mode_state
        if (msg.vehicle_mode_state == 0):#手动远程模式
            self.msgMap["carMode1"] = 1

        elif(msg.vehicle_mode_state == 1):#自动驾驶模式
             self.msgMap["carMode1"] = 2

        elif(msg.vehicle_mode_state == 2):#紧急模式
            self.msgMap["carMode1"] = 255

        elif(msg.vehicle_mode_state == 3):#待机模式
            self.msgMap["carMode1"] = 255

        # 车辆状态
        # "0x7: crash
        # 0x6: error
        # 0x5: E-stop
        # 0x4: work
        # 0x3: 3
        # 0x2: 2
        # 0x1: 1
        # 0x0: init
        # "

        #     表示车辆状态，有未知
        # （FF），未启动(1)，停车
        # （2）或驾驶中(3)四种，数
        # 值类型如：1，未知为 255

        # if (msg.CarWork_State == 4):#
        #     self.msgMap["carStatus"] = 3
        
        # elif (msg.CarWork_State == 5):#
        #     self.msgMap["carStatus"] = 2
        # else:
        #     self.msgMap["carStatus"] = 255
        
        # 车辆纵向，横向加速度
        self.msgMap["VehLongAccell"] = msg.acc#车辆加速度
        
        #刹车灯
        self.msgMap["BrakeLightSwitchSt"] = msg.brake_light_actual

    def brake_report_callback(self,msg):
        self.msgMap["braking"] = msg.brake_pedal_actual

    def gear_report_callback(self,msg):
        if (msg.gear_actual == 1):
            self.msgMap["gear"] = 80
        
        elif (msg.gear_actual == 2):
            self.msgMap["gear"] = 82
        
        elif (msg.gear_actual == 3):
            self.msgMap["gear"] = 78
            
        elif (msg.gear_actual == 4):
            self.msgMap["gear"] = 68
        else:
            self.msgMap["gear"] = 0xff

    def steering_report_callback(self,msg):
        if(msg != None):
            self.msgMap["SWA"] = msg.steer_angle_actual
            self.msgMap["SterringAngleSpd"] = msg.steer_angle_spd_actual

        else:
            self.msgMap["SWA"] = 0
            self.msgMap["SterringAngleSpd"] = 0
            
        return 1

    def throttle_report_callback(self,msg):
        self.msgMap["throttle"] = msg.throttle_pedal_actual
        return 1

#0-invaild 1-close_door 2-open_door
    def vehicle_command_callback(self,msg):
        if (msg.door_status_ctrl ==1 ):
            self.msgMap["DriverDoorLockSt"] = 1
        elif (msg.door_status_ctrl ==2 ):
            self.msgMap["DriverDoorLockSt"] = 0
        else:
            self.msgMap["DriverDoorLockSt"] = 255

    def lon_lat_callback(self, msg):
        self.msgMap["longitude"] = msg.longitude
        self.msgMap["latitude"] = msg.latitude
        self.msgMap["altitude"] = msg.altitude

    def Imu_callback(self,msg):
        (r, p, y) = transformations.euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        self.msgMap["vehiclePosture"]["postureX"]= p
        self.msgMap["vehiclePosture"]["postureY"]= r
        self.msgMap["vehiclePosture"]["postureZ"]= y
        self.msgMap["VehLatrlAccel"] = msg.linear_acceleration.y

if __name__ == "__main__":
    rospy.init_node("obu_chassis_RealTimeInformationUpload")
    RealTimeInformationUpload()
    rospy.spin()