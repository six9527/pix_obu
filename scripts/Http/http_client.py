# client.py
#!/usr/bin/python
#encoding:utf-8
import rospy
import requests
import json
from hashlib import sha256
import hmac
import base64
class HTTP_XLL :
    def __init__(self):
        self.url = "https://www.baidu.com"
        self.headers={"content-type" :"application/json;charset=utf-8"}
        self.app_secret = "njkhk2384cs542sc"
        self.payload = {
                "version": "1.0.0",
                "requestTime": None,
                "requestId": "1b930f7a5c41459091961cb7c1b624da",
                "appId": "CUPBOARD_12312312312",
                "sign": None,
                "command": "CUPBOARD_QUERY_BUSINESS_ORDER",
                "data": "{\"productKey\":\"xll_cupboard\",\"deviceName\":\"test-xiliulou-001\",\"orderId\":\"21312331231223231124\"}"
            }

    
    def getSign(self,data):
        sign_map = {
        'appId': data['appId'],
        'requestTime': data['requestTime'],
        'command': data['command'],
        'data': data['data']
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



if __name__=="__main__":
    rospy.init_node("http_xll")
    http = HTTP_XLL()
    print(http.getSign(http.payload))
    r = requests.post(http.url, data=json.dumps(http.payload), headers= http.headers)
    print (r.text)
    rospy.spin()