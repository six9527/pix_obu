#!/usr/bin/env python
# -*- coding:utf-8 -*- 
import rospy 
import json
from obu.srv import *
from pix_driver_msgs.msg import brake_report_501
from pix_driver_msgs.msg import gear_report_503
from pix_driver_msgs.msg import steering_report_502
from pix_driver_msgs.msg import vcu_report_505
from pix_driver_msgs.msg import park_report_504
from pix_driver_msgs.msg import throttle_report_500
from sensor_msgs.msg import NavSatFix, Imu
from tf import transformations


class RealTimeInformationUpload:
    def __init__(self):
        self.server = rospy.Service("/obu/chassis/RealTimeInformationUpload",OBUinterflowVehicle, self.callback_doReq)

        #转向，车速，启动状态，驾驶模式，车辆状态，加速度，刹车灯
        self.sub_vcu_report = rospy.Subscriber('/pix/vcu_report', vcu_report_505, self.vcu_report_callback)
        #刹车
        self.sub_brake_report = rospy.Subscriber("/pix/break_report",brake_report_501,self.brake_report_callback)
        #倒车
        self.sub_park_report = rospy.Subscriber("/pix/park_report",park_report_504,self.park_report_callback)
        #档位状态
        self.sub_gear_report = rospy.Subscriber("/pix/gear_report",gear_report_503,self.gear_report_callback)
        #方向盘转角，转角速度
        self.sub_steering_report = rospy.Subscriber("/pix/steering_report",steering_report_502,self.steering_report_callback)
        #油门状态
        self.sub_throttle_report = rospy.Subscriber("/pix/throttle_report",throttle_report_500,self.throttle_report_callback)


        #         #转向，车速，启动状态，驾驶模式，车辆状态，加速度，刹车灯
        # self.sub_vcu_report = rospy.Subscriber('/pix/vcu_report', VcuReport, self.vcu_report_callback)
        # #刹车
        # self.sub_brake_report = rospy.Subscriber("/pix/break_report",BrakeReport,self.brake_report_callback)
        # #倒车
        # self.sub_park_report = rospy.Subscriber("/pix/park_report",ParkReport,self.park_report_callback)
        # #档位状态
        # self.sub_gear_report = rospy.Subscriber("/pix/gear_report",GearReport,self.gear_report_callback)
        # #方向盘转角，转角速度
        # self.sub_steering_report = rospy.Subscriber("/pix/steering_report",SteerReport,self.steering_report_callback)
        # #油门状态
        # self.sub_throttle_report = rospy.Subscriber("/pix/throttle_report",ThrottleReport,self.throttle_report_callback)
        # ？？？？
        # 经纬度，海拔，车身姿态，定位状态，从车速度和距离，电机功率转速，横向纵向加速度，住车灯，刹车灯，倒车灯？？
        self.sub_lon_lat = rospy.Subscriber("/fixposition/navsatfix", NavSatFix, self.lon_lat_callback)#经纬度
        # /sensing/imu/imu_data
        self.sub_imu= rospy.Subscriber("/fixposition/poiimu", Imu, self.Imu_callback)#imu数据

        self.msgMap={
            "LocationType":31,
            #经纬度海拔
            "longitude":113.5951157,
            "latitude":22.74467066,
            "altitude":26.04,
            #定位精度
            "accuracy":1.0,

            "braking":0,
            "gear":"N",
            "speed":0,
            "SWA":0,
            "turnLight":0,
            #车身位姿，imu数据deg
            "vehiclePosture":{
                "postureX":255,
                "postureY":255,
                "postureZ":255},

            #从车位置
            "relativePosition":{
                "speed2":-999,
                "distanceR0":-999,
                "distanceRY":-999,
                "distanceRX":-999,
                "distance0":-999},

            "ifCarOnline":0,
            "throttle":0,
            #电机转速
            "rotationRate":-999,

            #电机功率
            " power":-999,

            "carMode1":2,
            "carStatus":3,
            "SterringAngleSpd":0,

            # 车辆纵向，横向加速度
            "VehLatrlAccel":-999,
            "VehLongAccell":-999,

            #驾驶位门锁
            "DriverDoorLockSt":255,

            
            "BrakeLightSwitchSt":0,
            "ParkingLampSt":255,

            #住车灯，倒车灯
            "Reverse_light_status":255
        }

    def park_report_callback(self,msg):
        # self.msgMap["ParkingLampSt"] = msg.Parking_Actual
        self.msgMap["ParkingLampSt"] = msg.Parking_Actual


    def callback_doReq(self, req):
        # for line in self.msgMap:
        #     msg = line.strip("\n")
        return json.dumps(self.msgMap,ensure_ascii=False)

    def vcu_report_callback(self,msg):
        # self.msgMap["speed"] = msg.Vehicle_Speed
        self.msgMap["speed"] = msg.Vehicle_Speed
        self.msgMap["turnLight"] =msg.TurnLight_Actual

        #启动判断
        if ( msg.CarPower_State,16 < 2):
            # 0不再线1在线
            self.msgMap["ifCarOnline"] = msg.CarPower_State
        else:
            self.msgMap["ifCarOnline"] = 0

        
        # 模式判断
        # 0x3: Standby Mode
        # 0x2: Emergency Mode
        # 0x1: Auto Mode
        # 0x0: Manual Remote Mode
        # msg.Vehicle_ModeState 
        #msg.vehicle_mode_state
        if (msg.Vehicle_ModeState,16 == 0):#手动远程模式
            self.msgMap["carMode1"] = 3

        elif(msg.Vehicle_ModeState,16 == 1):#自动驾驶模式
             self.msgMap["carMode1"] = 2

        elif(msg.Vehicle_ModeState,16 == 2):#紧急模式
            self.msgMap["carMode1"] = 255

        elif(msg.Vehicle_ModeState,16 == 3):#待机模式
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

        if (msg.CarWork_State,16 == 4):#
            self.msgMap["carStatus"] = 3
        
        elif (msg.CarWork_State,16== 5):#
            self.msgMap["carStatus"] = 2
        else:
            self.msgMap["carStatus"] = 255
        
        # 车辆纵向，横向加速度
        self.msgMap["VehLongAccell"] = msg.Vehicle_Acc#车辆加速度
        # self.msgMap["VehLongAccell"] = -999
        self.msgMap["VehLatrlAccel"] = -999
        #刹车灯
        self.msgMap["BrakeLightSwitchSt"] = msg.Brake_LightActual

    def brake_report_callback(self,msg):
        self.msgMap["braking"] = msg.Brake_PedalActual

    def gear_report_callback(self,msg):
        if (msg.Gear_Actual == 1):
            self.msgMap["gear"] = "P"
        
        elif (msg.Gear_Actual == 2):
            self.msgMap["gear"] = "R"
        
        elif (msg.Gear_Actual == 3):
            self.msgMap["gear"] = "N"

        elif (msg.Gear_Actual == 4):
            self.msgMap["gear"] = "D"
        else:
            self.msgMap["gear"] = 0xff

    def steering_report_callback(self,msg):
        if(msg != None):
            self.msgMap["SWA"] = msg.Steer_AngleActual
            self.msgMap["SterringAngleSpd"] = msg.Steer_AngleActual

        else:
            self.msgMap["SWA"] = -999
            self.msgMap["SterringAngleSpd"] = -999
            
        return 1

    def throttle_report_callback(self,msg):
        self.msgMap["throttle"] = msg.Dirve_ThrottlePedalActual
        return 1

    def lon_lat_callback(self, msg):
        self.msgMap["longitude"] = msg.longitude
        self.msgMap["latitude"] = msg.latitude
        self.msgMap["altitude"] = msg.altitude

    def Imu_callback(self,msg):
        (r, p, y) = transformations.euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        self.msgMap["vehiclePosture"]["postureX"]= p
        self.msgMap["vehiclePosture"]["postureY"]= r
        self.msgMap["vehiclePosture"]["postureZ"]= y

if __name__ == "__main__":
    rospy.init_node("obu_chassis_RealTimeInformationUpload")
    RealTimeInformationUpload()
    rospy.spin()