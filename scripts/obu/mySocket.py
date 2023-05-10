#!/usr/bin/env python
# -*- coding:utf-8 -*- 
import socket
import struct
import os
import config as msgConfig
import json
import yaml


import sys   #reload()之前必须要引入模块
reload(sys)
sys.setdefaultencoding('utf-8')

class UdpSocket(object):
    """
    socket通讯, 上传信息和下载信息
    """
    def __init__(self, addr, udp_socket_obj):
        self.addr=addr
        self.udp_socket=udp_socket_obj
        # self.udp_socket.setblocking(False)
        # self.udp_socket.settimeout(0.0)
        # 加载日志模块
        # logging.config.fileConfig(msgConfig.configFile_dir+"/socket.conf")
        # # self.root_logger = logging.getLogger()
        # self.send_app_logger = logging.getLogger('send_applog')
        # self.recv_app_logger = logging.getLogger('recv_applog')
        # self.error_app_logger = logging.getLogger('error_applog')





    @staticmethod
    def __data_fotmat(msg_body_len):
        #
        format_touple = (msgConfig.msg_format_prefix, str(msg_body_len), "s", msgConfig.msg_format_suffix) 
        return "".join(format_touple)

    # 数据打包成大端字节流的网络数据。computer string data 计算机数据
    def construct_byte(self, cmd, msg_body):
        msg_body_len =len(msg_body)
        communication_format = self.__data_fotmat(msg_body_len)
        return struct.pack(communication_format, cmd, msg_body_len,  msg_body.encode('utf-8'))
        

    # 网络数据解包成元组, Ndata-网络数据
    def deconstruct_message(self, Ndata):
        communication_format = self.__data_fotmat(len(Ndata)-msgConfig.msg_prefix_suffix_len)
        return struct.unpack(communication_format, Ndata)

    # 单次发生已经打包好的数据
    def single_send(self, cmd, send_str):
        """
            cmd 命令码--uint_8
            send_str 发送的字符串
        """
        # send_str = json.dumps(msg)
        send_data = self.construct_byte(cmd, send_str)
        self.udp_socket.sendto(send_data, self.addr)

    # 单次接受已经解包的数据
    def single_recv(self):
        data, addr = self.udp_socket.recvfrom(4096)
        recv_msg = self.deconstruct_message(data)
        # msg_body = json.loads(recv_msg[2])
        # print(msg_body["vin"])
        return recv_msg


def main():
    # rospy.init_node("tcp_socket_node")
    # 获取服务器和服务器端口
    addr=(msgConfig.server_ip, msgConfig.server_port)
    # 创建udp套接字
    udp_socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    # udp_test = UdpSocket(addr, udp_socket)
    udp_socket.close()

if __name__ == "__main__":
    main()