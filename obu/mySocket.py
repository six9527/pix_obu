#!/usr/bin/env python3
# -*- coding:utf-8 -*- 
import socket
import struct
import json
import os
current_file_path = os.path.abspath(__file__)
current_directory_path = os.path.dirname(current_file_path)
import sys
sys.path.append(current_directory_path)
import config

class UdpSocket(object):
    """
    socket通讯, 上传信息和下载信息
    """
    def __init__(self, server_address):
        self.server_address = server_address
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    @staticmethod
    def __data_fotmat(msg_body_len):
        #
        format_touple = (config.msg_format_prefix, str(msg_body_len), "s", config.msg_format_suffix) 
        return "".join(format_touple)

    # 数据打包成大端字节流的网络数据。computer string data 计算机数据
    def construct_byte(self, cmd, msg_body):
        msg_body_len =len(msg_body)
        communication_format = self.__data_fotmat(msg_body_len)
        return struct.pack(communication_format, cmd, msg_body_len,  msg_body.encode('utf-8'))
        

    # 网络数据解包成元组, Ndata-网络数据
    def deconstruct_message(self, Ndata):
        communication_format = self.__data_fotmat(len(Ndata)-config.msg_prefix_suffix_len)
        return struct.unpack(communication_format, Ndata)

    # 单次发生已经打包好的数据
    def single_send(self, cmd, send_str):
        # date = json.dumps(send_str)
        send_data = self.construct_byte(cmd, send_str)
        self.sock.sendto(send_data, self.server_address)

    # 单次接受已经解包的数据
    def single_recv(self):
        receive_data, client = self.sock.recvfrom(4096)
        recv_msg = self.deconstruct_message(receive_data)
        return recv_msg
    def close(self):
        self.sock.close()