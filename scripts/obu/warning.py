#!/usr/bin/env python
# -*- coding:utf-8 -*- 
from cgi import test
import imghdr
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from cv_bridge import CvBridge, CvBridgeError
import cv2
from autoware_perception_msgs.msg import Warning
from jsk_rviz_plugins.msg import OverlayText
import re
import numpy as np

def str2int(s,v):
    try:
        r=int(s,16)
    except:
        r=v
    return r

def parse_rgba(s):
    r=str2int(s[0:2],0)
    g=str2int(s[2:4],0)
    b=str2int(s[4:6],0)
    a=str2int(s[6:8],255)
    return (r,g,b,a)


def check_hex(s,v):
    p = re.compile(r'\b([0-9a-fA-F]+)\b')
    m = re.search(p, s)
    if m:
        return m.group(1)
    else:
        return v


def complement_msg(msg, warning_suggestion):
    ot_action= 0
    ot_width= 360
    ot_height= 600
    ot_left= 0
    ot_top= 0
    ot_bg_color= '00000010'
    ot_line_width= 360
    ot_text_size= 40
    ot_font= "黑体"
    ot_fg_color= '2878c5c0'
    #05179fff 蓝
    #2878c5ff 兰
    #C08001ff 土黄
    ot_bg_color= check_hex(ot_bg_color,'00000010')
    ot_fg_color=  check_hex(ot_fg_color,'2878c5c0')
    r,g,b,a= parse_rgba(ot_bg_color)
    ot_bg_color_r = float(r)/255.0
    ot_bg_color_g = float(g)/255.0
    ot_bg_color_b = float(b)/255.0
    ot_bg_color_a = float(a)/255.0
    r,g,b,a = parse_rgba( ot_fg_color)
    ot_fg_color_r = float(r)/255.0
    ot_fg_color_g = float(g)/255.0
    ot_fg_color_b = float(b)/255.0
    ot_fg_color_a = float(a)/255.0
    
    msg.action = ot_action
    msg.width = ot_width
    msg.height = ot_height
    msg.left = ot_left
    msg.top = ot_top
    msg.bg_color.r = ot_bg_color_r
    msg.bg_color.g = ot_bg_color_g
    msg.bg_color.b = ot_bg_color_b
    msg.bg_color.a = ot_bg_color_a
    msg.line_width = ot_line_width
    msg.text_size = ot_text_size
    msg.font = ot_font
    msg.fg_color.r = ot_fg_color_r
    msg.fg_color.g = ot_fg_color_g
    msg.fg_color.b = ot_fg_color_b
    msg.fg_color.a = ot_fg_color_a
    msg.text = warning_suggestion
    
    rviz_text_pub.publish(msg)

def rviz_text_puber(warning_suggestion):
    msg=OverlayText()
    complement_msg(msg, warning_suggestion)

def camera_callback(msg):
    global test_sub
    test_sub = 1
    if msg.warningType == 1:
        #rospy.loginfo("前方碰撞")
        warning_suggestion = "前方碰撞"
        image_path = '/home/t/ros_driver/pix_obu/src/pix_obu/pix_obu/obu_picture/'+'1.png'
    elif msg.warningType == 2:
        #rospy.loginfo("路口碰撞")
        warning_suggestion = "路口碰撞"
        image_path = '/home/t/ros_driver/pix_obu/src/pix_obu/pix_obu/obu_picture/'+'2.png'
    elif msg.warningType == 3:
        #rospy.loginfo("左转辅助")
        warning_suggestion = "左转辅助"
        image_path = '/home/t/ros_driver/pix_obu/src/pix_obu/pix_obu/obu_picture/'+'3.png'
    elif msg.warningType == 4:
        #rospy.loginfo("盲区预警")
        warning_suggestion = "盲区预警"
        image_path = '/home/t/ros_driver/pix_obu/src/pix_obu/pix_obu/obu_picture/'+'4.png'
    elif msg.warningType == 5:
        #rospy.loginfo("逆向超车")
        warning_suggestion = "逆向超车"
        image_path = '/home/t/ros_driver/pix_obu/src/pix_obu/pix_obu/obu_picture/'+'5.png'
    elif msg.warningType == 6:
        #rospy.loginfo("异常车辆")
        warning_suggestion = "异常车辆"
        image_path = '/home/t/ros_driver/pix_obu/src/pix_obu/pix_obu/obu_picture/'+'6.png'
    elif msg.warningType == 7:
        #rospy.loginfo("紧急刹车")
        warning_suggestion = "紧急刹车"
        image_path = '/home/t/ros_driver/pix_obu/src/pix_obu/pix_obu/obu_picture/'+'7.png'
    elif msg.warningType == 8:
        #rospy.loginfo("车辆失控")
        warning_suggestion = "车辆失控"
        image_path = '/home/t/ros_driver/pix_obu/src/pix_obu/pix_obu/obu_picture/'+'8.png'
    elif msg.warningType== 9:
        #rospy.loginfo("紧急车辆")
        warning_suggestion = "紧急车辆"
        image_path = '/home/t/ros_driver/pix_obu/src/pix_obu/pix_obu/obu_picture/'+'9.png'
    elif msg.warningType == 10:
        #rospy.loginfo("车内标牌")
        warning_suggestion = "车内标牌"
        image_path = '/home/t/ros_driver/pix_obu/src/pix_obu/pix_obu/obu_picture/'+'10.png'
    elif msg.warningType == 11:
        #rospy.loginfo("危险路段")
        warning_suggestion = "危险路段"
        image_path = '/home/t/ros_driver/pix_obu/src/pix_obu/pix_obu/obu_picture/'+'11.png'
    elif msg.warningType == 12:
        #rospy.loginfo("闯红灯")
        warning_suggestion = "闯红灯"
        image_path = '/home/t/ros_driver/pix_obu/src/pix_obu/pix_obu/obu_picture/'+'12.png'
    elif msg.warningType == 13:
        #rospy.loginfo("绿波引导")
        warning_suggestion = "绿波引导"
        image_path = '/home/t/ros_driver/pix_obu/src/pix_obu/pix_obu/obu_picture/'+'13.png'
    elif msg.warningType == 14:
        #rospy.loginfo("拥堵路段")
        warning_suggestion = "拥堵路段"
        image_path = '/home/t/ros_driver/pix_obu/src/pix_obu/pix_obu/obu_picture/'+'14.png'
    elif msg.warningType == 15:
        #rospy.loginfo("施工路段")
        warning_suggestion = "施工路段"
        image_path = '/home/t/ros_driver/pix_obu/src/pix_obu/pix_obu/obu_picture/'+'15.png'
    elif msg.warningType == 16:
        #rospy.loginfo("行人碰撞")
        warning_suggestion = "行人碰撞"
        image_path = '/home/t/ros_driver/pix_obu/src/pix_obu/pix_obu/obu_picture/'+'16.png'
    elif msg.warningType == 17:
        #rospy.loginfo("超速预警")
        warning_suggestion = "超速预警"
        image_path = '/home/t/ros_driver/pix_obu/src/pix_obu/pix_obu/obu_picture/'+'17.png'
    elif msg.warningType == 18:
        #rospy.loginfo("协同路口")
        warning_suggestion = "协同路口"
        image_path = '/home/t/ros_driver/pix_obu/src/pix_obu/pix_obu/obu_picture/'+'18.png'
    elif msg.warningType == 19:
        #rospy.loginfo("动态车道")
        warning_suggestion = "动态车道"
        image_path = '/home/t/ros_driver/pix_obu/src/pix_obu/pix_obu/obu_picture/'+'19.png'
    elif msg.warningType == 20:
        #rospy.loginfo("匝道汇入")
        warning_suggestion = "匝道汇入"
        image_path = '/home/t/ros_driver/pix_obu/src/pix_obu/pix_obu/obu_picture/'+'20.png'
    elif msg.warningType == 21:
        #rospy.loginfo("协作变道")
        warning_suggestion = "协作变道"
        image_path = '/home/t/ros_driver/pix_obu/src/pix_obu/pix_obu/obu_picture/'+'21.png'
    else:
        warning_suggestion = "无预警信息"
        image_path = '/home/t/ros_driver/pix_obu/src/pix_obu/pix_obu/obu_picture/'+'333.png'

    image_ = cv2.imread(image_path)
    img = np.array(image_)
    bridge = CvBridge()
    try:
        image_msg = bridge.cv2_to_imgmsg(img, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("Conversion error: {}".format(e))
    
    image_pub.publish(image_msg)

    rviz_text_puber(warning_suggestion)

def check_received(timer):
    global test_sub
    if test_sub == 1:
        pass
        # print(1)
    else:
        # print(0)
        msg=OverlayText()
        complement_msg(msg, "无预警信息")

        normal_img_path = "/home/t/ros_driver/pix_obu/src/pix_obu/pix_obu/obu_picture/333.png"

        img = cv2.imread(normal_img_path)
        bridge = CvBridge()
        try:
            image_msg = bridge.cv2_to_imgmsg(img, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("Conversion error: {}".format(e))
        
        image_pub.publish(image_msg)
        
    test_sub = 0

if __name__ == '__main__':
    global  test_sub
    test_sub = 0
    rospy.init_node('image_publisher')
    
    image_pub = rospy.Publisher('warning_image', Image, queue_size=10)
    rviz_text_pub = rospy.Publisher('/overlaytext_rviz', OverlayText, queue_size=1)
    
    
  
    warning_type_sub = rospy.Subscriber('/warning_type', Warning, camera_callback)

    rospy.Timer(rospy.Duration(1), check_received)
    rospy.spin()