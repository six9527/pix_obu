#!/usr/bin/env python
# -*- coding:utf-8 -*- 
"""
3.3.1.4 车载主机车辆异常信息     -- 触发           chassis abnormal Information report
 
"""
import rospy 
import json
from obu.srv import *
from pix_driver_msgs.msg import VcuReport
from pix_driver_msgs.msg import BrakeReport

class AbnormalInformationUpload:
    def __init__(self):
        self.server = rospy.Service("/obu/chassis/AbnormalInformationUpload",OBUinterflowVehicle, self.callback_doReq)
        self.sub_vcu_report = rospy.Subscriber('/pix/vcu_report', VcuReport, self.vcu_report_callback)
        self.sub_brake_report = rospy.Subscriber("/pix/break_report",BrakeReport,self.brake_report_callback)
        self.msgMap={
            "vehicleElectronicState":"0",
            "carAbnormal":1
        }
        # self.msgMap={"warningType":1,
        #              "signType":0,
        #              "evenType":0,
        #              "warningInfo":" 前 车 碰 撞 预 警",
        #              "warningSuggestion":"请减速，前方有碰撞危险",
        #              "distant":40}
        
    def callback_doReq(self, req):
        msg = json.dumps(self.msgMap, ensure_ascii=False)
        return msg

    def vcu_report_callback(self,msg):
        self.msgMap["vehicleElectronicState"] = str(msg.chassis_errcode)    

    def brake_report_callback(self,msg):
        if (msg.brake_flt2 == 1 or msg.brake_flt1 ==1 ):
            self.msgMap["carAbnormal"] = 64
    




if __name__ == "__main__":
    rospy.init_node("obu_chassis_AbnormalInformationUpload")
    AbnormalInformationUpload()
    rospy.spin()