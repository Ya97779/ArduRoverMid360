#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import struct
from mavros_msgs.msg import Mavlink
from std_msgs.msg import Float32MultiArray

class CustomMavlinkListener:
    def __init__(self):
        rospy.init_node('custom_mavlink_parser', anonymous=True)
        
        # 订阅 MAVROS 吐出的底层 MAVLink 原始字节流
        rospy.Subscriber('/mavlink/from', Mavlink, self.raw_mavlink_cb)
        
        # 将解析并换算后的真实物理数据，发布为 ROS 话题
        # 数组顺序规定为: [电压(V), 电流(A), 电量(%)]
        self.custom_pub = rospy.Publisher('/rover/custom_hex_240', Float32MultiArray, queue_size=10)
        
        # 自定义 MAVLink Message ID
        self.CUSTOM_MSG_ID = 240

    def raw_mavlink_cb(self, msg):
        # 1. 拦截 ID 为 240 的数据包
        if msg.msgid == self.CUSTOM_MSG_ID:
            
            # 2. 从 payload64 还原字节流
            raw_bytes = bytearray()
            for val in msg.payload64:
                raw_bytes.extend(struct.pack('<Q', val))
                
            # 3. MAVLink 2 防坑处理 (末尾去零补齐)
            valid_bytes = raw_bytes[:msg.len]
            if len(valid_bytes) < 3:
                valid_bytes.extend(b'\x00' * (3 - len(valid_bytes)))
            
            # 4. 开始解包与真实物理换算
            try:
                # 读出 3 个无符号整型字节
                byte1, byte2, byte3 = struct.unpack('<BBB', valid_bytes[:3])
                
                # ================= 核心换算逻辑 =================
                vol = byte1 / 10.0   # 电压除以 10
                cur = byte2 / 10.0   # 电流除以 10
                pct = float(byte3)   # 电量不需要除以 10，直接转为浮点数
                # ================================================
                
                # 打印到终端，方便后台监控排错
                rospy.loginfo(f" 解析成功: 电压 {vol}V | 电流 {cur}A | 电量 {pct}%")
                
                # 5. 打包发送给串口网关和行为树
                ros_msg = Float32MultiArray()
                ros_msg.data = [vol, cur, pct]
                self.custom_pub.publish(ros_msg)
                
            except struct.error as e:
                rospy.logerr(f"解包错误: {e}")

if __name__ == '__main__':
    try:
        listener = CustomMavlinkListener()
        rospy.loginfo("MAVLink 解包节点已启动，正在监听飞控电池数据 (msgid=240)...")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass