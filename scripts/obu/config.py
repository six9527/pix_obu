#!/usr/bin/env python
# -*- coding:utf-8 -*- 

# 服务器配置信息
server_ip="192.168.1.103"
# server_port=1111
# server_ip="127.0.0.1"
server_port=1247

"""
一般网络人为协议都是前缀+消息体+后缀
<小端  >大端
消息协议：命令码+消息体长度+消息体
命令码     1Byte
消息体长度 4Byte
消息体     NByte
"""
msg_format_prefix="<BI" 
msg_format_suffix=""
msg_cmd_len = 1
msg_Body_len = 4
msg_prefix_suffix_len=5
# 红绿灯对应表
traffic_light={"light219218":4898,"light218217":4999,"light215124":4887,"light213214":4876}#4887

# -------------------------------------------------------------
import os
configFile_dir = os.path.dirname(os.path.abspath(__file__))  # 上级目录
# /root/code/OBU_ws/src/obu_manage_system/Config

# loggingCfgPath = configFile_dir