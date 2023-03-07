#!/usr/bin/env python
# -*- coding:utf-8 -*- 
"""
3.3.1.1 车载主机信息注册     --  双向 一次           chassis Registration Information report
"""
import rospy 
import json
from obu.srv import OBUinterflowVehicleResponse, OBUinterflowVehicle, OBUinterflowVehicleRequest


class RegistrationInformationReport:
    def __init__(self):
        self.server = rospy.Service("/obu/chassis/RegistratIonInformationReport",OBUinterflowVehicle, self.callback_doReq)
        self.msgMap={
            # "vin_IN":"车辆标识,车架号, 字符类型",
            "vin":"3LN6L5SU9KR601288",

            # "plateNumber_IN":"车牌号, 字符类型。如:粤A0V08",
            "plateNumber":None,

            # "personInCharge_IN":"安全员名字",
            "personInCharge":"happiness",

            # "manufacturer_IN":"车辆厂商名字",
            "manufacturer":"PIXMOVING",

            # "company_IN":"无人驾驶公司名字",
            "company":"PIXMOVING",

            # "vehicleType_IN":"车辆类型 uint8 有未知(FF)纯电动(1),纯油 (2)，混合动力(3)四种",
            "vehicleType":1,

            # "vehicleProperty_IN":"车辆类型 uint8 有未知(FF) 测试(1) 运营(2)",
            "vehicleProperty":1,

            # "carType_IN":"车辆种类, A为轿车(5 座及以下)、B为小巴、C为中巴、D为大巴、E为货车、F为卡车、G挂车、H为物流车(包含外卖、快递等)、I 为清洁车、J 为巡逻车、K 为消毒车、L 为售卖车（销售 饮料等)",
            "carType":"H",

            # "brand_IN":"车辆品牌, 如:奥迪、阿尔法·罗密欧、奔驰、宝马、红旗、Jeep、捷豹、雷克萨斯、凯迪拉克、路 虎、林肯、讴歌、乔治·巴顿、沃尔沃、英菲尼迪……。 如:日产,注意不填时,brand为null,即不上报, 不要填字符串”null”、””、“none",
            "brand":None,

            # "series_IN":"车系,车型，如奥迪 a6中的a6, 林肯MKZ。如:MKZ",
            "series":None,

            # "equip_code_IN":"测试车企设备编码",
            "equip_code":"PIXMOVING001",

            # "equip_name_IN":"测试车企设备名称",
            "equip_name":"PIXMOVING"
        }
    
    def callback_doReq(self, req):
        msg = json.dumps(self.msgMap,ensure_ascii=False)
        return msg



if __name__ == "__main__":
    rospy.init_node("obu_chassis_RegistratIonInformationReport")
    RegistrationInformationReport()
    rospy.spin()