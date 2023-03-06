#!/usr/bin/python
#encoding:utf-8
import uuid
import time
import json
from hashlib import sha256
import hmac
import base64
import requests
import rospy

class API:
    def __init__(self):

        # self. URL = "http://127.0.0.1:8000"
        self. URL = "http://192.168.47.133:8000"
        # self. URL = "https://www.baidu.com/"
        self.headers={'content-type':'application/json', 'Connection':'close'}
        self.app_secret = "njsnhfu1684afaeff"
        self.version = "1.0.0"
        self.appId = "CUPBOARD_12312312312"
        self. productKey = "xll_cupboard" 
        self.deviceName = "test-xiliulou-001"

        self.post_data={
            "version": self.version,
            "requestTime": None,
            "requestId": None,
            "appId": self.appId,
            "sign": None,
            "command": None,
            "data":None
            }
    
    # 获取签名
    def getSign(self,data):
        sign_map = {
        "appId": data["appId"],
        "requestTime": data["requestTime"],
        "command": data["command"],
        "data": data["data"]
        }
        sign_map = sorted(sign_map.items(), key=lambda x: x[0])
        strp = ''
        for key in sign_map:
            strp = strp + str(key[0]) + '=' + str(key[1]) + ','
            s = strp.rstrip(',')
            sign = self.calSignature(s)
        return sign

    def calSignature(self, s):
        key = self.app_secret.encode('utf-8')
        message = s.encode('utf-8')
        sign = base64.urlsafe_b64encode(hmac.new(key, message, digestmod=sha256).digest())
        sign = str(sign).encode("utf-8")
        return sign

#python2使用json.loads将数据转为dict后单个数据还是unicode类型，需要经过该函数转换为是正常类型
    def unicode_convert(self,input):
        if isinstance(input, dict):
            return {self.unicode_convert(key): self.unicode_convert(value) for key, value in input.iteritems()}
        elif isinstance(input, list):
            return [self.unicode_convert(element) for element in input]
        elif isinstance(input, unicode):
            return input.encode('utf-8')
        else:
            return input

# 打包数据，获取签名，时间戳和请求ID等,发送post请求
    def data_pkg(self, data, command):
        self.post_data["requestTime"] = int(time.time())
        self.post_data["requestId"] = str(uuid.uuid1())#生成请求ID
        self.post_data["command"] = command
        self.post_data["data"] = data
        self.post_data["sign"] = self.getSign(self.post_data)
        result = requests.post(self.URL, json=self.post_data, headers= self.headers, verify=False,timeout=10)
        response_data= self.unicode_convert(json. loads(result.text))
        print(type(response_data["data"]["gmtOnline"]))
        if response_data["code"] == 1:
              rospy.logwarn(response_data["errCode"],response_data["errMsg"])
        return response_data["code"], response_data["data"]

# 硬件命令
    def hardware_command(self):
        data = {
                "productKey":self.productKey,
                "deviceName":self.deviceName,
                "hardwareCommand":"api_open_cell",
                "attrInfo":{"cellList":[1,2,3,4]}
                }
        command = "HARDWARE_COMMAND"
        code,response = self.data_pkg(data,command)
        if (code == 1):
            rospy.logwarn(response["errCode"],response["errMsg"])
        else:
             rospy.logwarn("操作成功！！！") 
             print(response)

# 设备状态查询
    def check_device_status(self):
        data = {
                    "productKey":self.productKey,
                    "deviceName":self.deviceName,
                    }
        command = "CHECK_DEVICE_STATUS"
        code,response = self.data_pkg(data,command)
        if (code == 1):
            rospy.logwarn(response["errCode"],response["errMsg"])
        else:
            #  gmtOnline = response["data"]["gmtOnline"]
            #  ipAddress = response["data"]["ipAddress"]
            #  online = response["data"]["online"]
             rospy.logwarn("操作成功！！！") 
             print(response)


# 柜机信息查询
    def  query_cupboard_info(self):
        command = "QUERY_CUPBOARD_INFO"
        data = {
                    "productKey":self.productKey,
                    "deviceName":self.deviceName,
                    }
        code,response = self.data_pkg(data,command)
        if (code == 1):
            rospy.logwarn(response["errCode"],response["errMsg"])
        else:
            print(response)

# 查询格挡信息
    def query_cell(self):
        command = "QUERY_CELL"
        data = {
                    "productKey":self.productKey,
                    "deviceName":self.deviceName,
                    "type": 2, #查询类型，1--全部，2--指定
                    "cellList":[1,2,3,4,5] #如果type为2，那么就必须传这个参数
                    }
        code,response = self.data_pkg(data,command)
        if (code == 1):
            rospy.logwarn(response["errCode"],response["errMsg"])
        else:
            print(response)

# 西六楼系统分配格挡，并发开门指令，业务存餐目前不支持存入预约订单
    def cupboard_put_meal(self):
        command = "CUPBOARD_PUT_MEAL"
        data = {
                    "productKey":self.productKey,
                    "deviceName":self.deviceName,
                    "orderId":"123456789",#第三方订单编号，请保证唯一，长度不可以超过40，值为存餐订单号。
                    "boxSize":1,#格挡大小，0--小格， 1--中格， 2--大格
                    "pickCode":"1231",#取餐码
                    "openHeat":1,
                    "openDisinfect":1,#是否开启消毒：0--不开，1--开启
                    "openLight":1
                    }
        code,response = self.data_pkg(data,command)
        if (code == 1):
            rospy.logwarn(response["errCode"],response["errMsg"])
        else:
            print(response)

    # 西六楼系统分配占用格挡,但是不执行开门命令,会生成订单
    def  cupboard_occupy_meal(self):
        command = "CUPBOARD_OCCUPY_MEAL"
        data = {
                     "productKey":self.productKey,
                    "deviceName":self.deviceName,
                    "orderId":"123456789",
                    "boxSize":1,
                    "pickCode":"1231"
                    }
        code,response = self.data_pkg(data,command)
        if (code == 1):
            rospy.logwarn(response["errCode"],response["errMsg"])
        else:
            print(response)

# 西六楼系统取消订单，并发开门指令
    def cupboard_cancel_meal(self):
        command = "CUPBOARD_CANCEL_MEAL"
        data = {
                     "productKey":self.productKey,
                    "deviceName":self.deviceName,
                    "orderId":"123456789",#第三方订单编号，请保证唯一，长度不可以超过40，值为存餐订单号。
                    }
        code,response = self.data_pkg(data,command)
        if (code == 1):
            rospy.logwarn(response["errCode"],response["errMsg"])
        else:
            print(response)


# 取餐，西六楼系统完成订单并会自动关闭加热，消毒，灯。
    def cupboard_take_meal(self):
        command = "CUPBOARD_TAKE_MEAL"
        data = {
                     "productKey":self.productKey,
                    "deviceName":self.deviceName,
                    "orderId":"123456789",#第三方订单编号，请保证唯一，长度不可以超过40，值为存餐订单号。
                    }
        code,response = self.data_pkg(data,command)
        if (code == 1):
            rospy.logwarn(response["errCode"],response["errMsg"])
        else:
            print(response)

# 换餐，当骑手存错餐品时，可以临时开门进行换餐
    def  cupboard_temp_open(self):
        command = "CUPBOARD_TAKE_MEAL"
        data = {
                     "productKey":self.productKey,
                    "deviceName":self.deviceName,
                    "orderId":"123456789",#第三方订单编号，请保证唯一，长度不可以超过40，值为存餐订单号。
                    }
        code,response = self.data_pkg(data,command)
        if (code == 1):
            rospy.logwarn(response["errCode"],response["errMsg"])
        else:
            print(response)


# 根据第三方订单Id查询西六楼系统的业务订单信息
    def  cupboard_query_business_order(self, orderId = "123456789"):
        command = "CUPBOARD_QUERY_BUSINESS_ORDER"
        data = {
                    "productKey":self.productKey,
                    "deviceName":self.deviceName,
                    "orderId":orderId,#第三方订单编号，请保证唯一，长度不可以超过40，值为存餐订单号。
                    }
        code,response = self.data_pkg(data,command)
        if (code == 1):
            rospy.logwarn(response["errCode"],response["errMsg"])
        else:
            print(response)

# 释放格挡占用状态或者锁定解锁格挡
    def cupboard_cell_update(self):
        command = "CUPBOARD_CELL_UPDATE"
        data = {
                        "productKey":self.productKey,
                        "deviceName":self.deviceName,
                        "type":2,
                        "blockType":1,
                        "cellList":[12,3,4,5],
                        "releaseOrder":1,#第三方订单编号，请保证唯一，长度不可以超过40，值为存餐订单号。
                    }
        code,response = self.data_pkg(data,command)
        if (code == 1):
            rospy.logwarn(response["errCode"],response["errMsg"])
        else:
            print(response)

# 根据要查询的请求的requestId来查询当时的调用记录
    def query_request_record(self,requestId):
        command = "QUERY_REQUEST_RECORD"
        data = {
                    "productKey":self.productKey,
                    "deviceName":self.deviceName,
                    "requestId":requestId
                    }
        code,response = self.data_pkg(data,command)
        if (code == 1):
            rospy.logwarn(response["errCode"],response["errMsg"])
        else:
            print(response)

if __name__=="__main__":
    api = API()
    api.query_cupboard_info()
    time.sleep(1)
    api.check_device_status()
    time.sleep(1)
    api.query_cell()
    time.sleep(1)
    api.cupboard_query_business_order()

