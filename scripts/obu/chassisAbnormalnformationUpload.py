#!/usr/bin/env python
# -*- coding:utf-8 -*- 
"""
3.3.1.4 车载主机车辆异常信息     -- 触发           chassis abnormal Information report
 
"""
import rospy 
import json
from obu.srv import *


class AbnormalInformationUpload:
    def __init__(self):
        self.server = rospy.Service("/obu/chassis/AbnormalInformationUpload",OBUinterflowVehicle, self.callback_doReq)

        self.msgMap={
            "vehicleElectronicState":"",
            "carAbnormal":1

# "lightDetails":[{"if_current_light":1,"type":0,"state":0,"timeLeft":20},{"if_current_light":1,"type":1, "state":2,"timeLeft":30},{"if_current_light":1,"type":2,"state":2,"timeLeft":30},{"if_current_light":0, "type":3,"state":2,"timeLeft":4}]
        }
        

    
    def callback_doReq(self, req):
        msg = json.dumps(self.msgMap, ensure_ascii=False)
        return msg



if __name__ == "__main__":
    rospy.init_node("obu_chassis_AbnormalInformationUpload")
    AbnormalInformationUpload()
    rospy.spin()