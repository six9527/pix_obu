#!/usr/bin/env python3
# -*- coding:utf-8 -*- 

# 服务器配置信息
# server_ip="192.168.1.103"
# server_port=1111
server_ip="127.0.0.1"
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
traffic_light={"light219218":"4898","light218217":"4999","light215124":"4887","light213214":"4876"}#4887

# -------------------------------------------------------------
from pix_sweeping_driver_msgs.msg import BmsReport as BmsReport
from pix_sweeping_driver_msgs.msg import BmsReport as BmsReport
from pix_sweeping_driver_msgs.msg import BrakeReport as BrakeReport
from pix_sweeping_driver_msgs.msg import GearReport as GearReport
from pix_sweeping_driver_msgs.msg import ParkReport as ParkReport
from pix_sweeping_driver_msgs.msg import SteeringReport as SteeringReport
from pix_sweeping_driver_msgs.msg import ThrottleReport as ThrottleReport
from pix_sweeping_driver_msgs.msg import VcuReport as VcuReport
from pix_sweeping_driver_msgs.msg import WheelSpeedReport as WheelSpeedReport

BmsTopic = '/pix_sweeping/bms_report'
BrakeTopic = '/pix_sweeping/brake_report'
GearTopic = '/pix_sweeping/gear_report'
ParkTopic = '/pix_sweeping/park_report'
SteeringTopic = '/pix_sweeping/steering_report'
ThrottleTopic = '/pix_sweeping/throttle_report'
VcuTopic = '/pix_sweeping/vcu_report'
WheelSpeedTopic = "/pix_sweeping/wheel_speed_report"
import os
configFile_dir = os.path.dirname(os.path.abspath(__file__))  # 上级目录