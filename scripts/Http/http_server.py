from http.server import HTTPServer, BaseHTTPRequestHandler
import json
import time

data = {'result': 'this is a test'}
host = ('192.168.47.133', 8000)

def unicode_convert(input):
    if isinstance(input, dict):
        return {unicode_convert(key): unicode_convert(value) for key, value in input.iteritems()}
    elif isinstance(input, list):
        return [unicode_convert(element) for element in input]
    elif isinstance(input, unicode):
        return input.encode('utf-8')
    else:
        return input

class Resquest(BaseHTTPRequestHandler):
    def do_GET(self):
        self.send_response(200)
        self.send_header('Content-type', 'application/json')
        self.end_headers()
        self.wfile.write(json.dumps(data).encode())

    def do_POST(self):
        datas = self.rfile.read(int(self.headers['content-length']))
        request_data = unicode_convert(json.loads(datas))
        if  request_data["command"]  == "CHECK_DEVICE_STATUS":
            print(request_data)
        # print(datas.headers)
        # data = json.loads(datas,encoding='utf-8')
        # print(unicode_convert(data),type(unicode_convert(data)))
        # print('headers', self.headers)
        # print("do post:", self.path, self.client_address, datas)
        data = {
            "code": 0,
            "sysTime": time.time(),
            "errCode": "",
            "errMsg": "",
            "data":{
                "gmtOnline": "2021-08-29 09:39:11",
                "ipAddress": "39.144.6.183",
                "online": True
                },
            }
        self.send_response(200)
        self.send_header('Content-type', 'application/json')
        self.end_headers()
        self.wfile.write(json.dumps(data).encode())

if __name__ == '__main__':
    server = HTTPServer(host, Resquest)
    print("Starting server, listen at: %s:%s" % host)
    server.serve_forever()
