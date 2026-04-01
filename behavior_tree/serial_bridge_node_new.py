#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import serial
import json
import threading
from std_msgs.msg import String, Bool, Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPointStamped
from actionlib_msgs.msg import GoalID
from mavros_msgs.srv import CommandBool, SetMode, CommandHome

# ================= 配置区域 =================
SERIAL_PORT = '/dev/ttyGS0'  # '/dev/ttyGS0  /dev/pts/0'
BAUD_RATE = 115200

# 飞控 HOME 点固定坐标
FIXED_HOME_LAT = 1.3000000
FIXED_HOME_LON = 103.8000000
FIXED_HOME_ALT = 20.0
# ==========================================

# 存放机器人全局状态的字典（准备发给 Windows 的 JSON 原型）
robot_state = {
    "pose_x": 0.0,
    "pose_y": 0.0,
    "vol": 0.0,
    "cur": 0.0,
    "pct": 0.0,
    "task_status": "Idle",
}

class SerialBridge:
    def __init__(self):
        rospy.init_node('serial_bridge_node', anonymous=True)
        
        # 1. 初始化串口
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
            rospy.loginfo(f"串口 {SERIAL_PORT} 打开成功，等待上位机连接...")
        except Exception as e:
            rospy.logerr(f"串口打开失败: {e}\n请确认是否已执行 sudo chmod 666 {SERIAL_PORT}")
            return

        # 2. 状态监听 (订阅底层 ROS 数据)
        rospy.Subscriber('/Odometry', Odometry, self.odom_cb) 
        rospy.Subscriber('/robot/current_state', String, self.bt_status_cb)
        
        # 【核心对接】订阅独立解包节点发来的、已经除以10的真实物理数据
        rospy.Subscriber('/rover/custom_hex_240', Float32MultiArray, self.custom_battery_cb)
        
        # 3. 指令发布与服务代理 (用于执行上位机指令)
        self.cmd_pub = rospy.Publisher('/commander/cmd', String, queue_size=1)
        self.patrol_pub = rospy.Publisher('/commander/patrol_enable', Bool, queue_size=1)
        self.autocharge_pub = rospy.Publisher('/commander/autocharge_enable', Bool, queue_size=1, latch=True)
        
        # Actionlib 取消目标的话题 (用于 STOP_TASK)
        self.move_base_cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
        self.autodock_cancel_pub = rospy.Publisher('/autodock_action/cancel', GoalID, queue_size=1)

        # MAVROS 服务
        self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.home_service = rospy.ServiceProxy('/mavros/cmd/set_home', CommandHome)
        self.origin_pub = rospy.Publisher('/mavros/global_position/set_gp_origin', GeoPointStamped, queue_size=1, latch=True)

        # 4. 开启后台接收线程与定时发送
        threading.Thread(target=self.rx_thread, daemon=True).start()
        
        # 2Hz 定时器：每 0.5 秒发一次状态给 Windows
        rospy.Timer(rospy.Duration(0.5), self.tx_timer_cb) 

    # ================= 状态更新回调函数 =================
    def odom_cb(self, msg):
        robot_state["pose_x"] = round(msg.pose.pose.position.x, 2)
        robot_state["pose_y"] = round(msg.pose.pose.position.y, 2)

    def bt_status_cb(self, msg):
        robot_state["task_status"] = msg.data

    def custom_battery_cb(self, msg):
        """接收并更新电池数据（不需要做数学运算，直接保留两位小数即可）"""
        if len(msg.data) >= 3:
            robot_state["vol"] = round(msg.data[0], 2)
            robot_state["cur"] = round(msg.data[1], 2)
            robot_state["pct"] = round(msg.data[2], 2)

    # ================= 定时发送线程 (Jetson -> Windows) =================
    def tx_timer_cb(self, event):
        try:
            if self.ser.is_open:
                # 把装满真实数据的 robot_state 字典变成 JSON 字符串，加上换行符发往 Windows
                msg = json.dumps(robot_state) + '\n'
                self.ser.write(msg.encode('utf-8'))
        except Exception:
            pass 

    # ================= 循环接收线程 (Windows -> Jetson) =================
    def rx_thread(self):
        while not rospy.is_shutdown():
            # 1. 检查并重连串口 (如果串口丢失或关闭，则不断尝试重新打开)
            if self.ser is None or not self.ser.is_open:
                try:
                    # 注意：如果你的全局变量叫 SERIAL_PORT，请把 '/dev/ttyGS0' 换成 SERIAL_PORT
                    self.ser = serial.Serial('/dev/ttyGS0', 115200, timeout=0.1)
                    rospy.loginfo("串口 /dev/ttyGS0 打开成功，等待上位机连接...")
                except serial.SerialException:
                    rospy.sleep(1.0)  # 如果线还没插上，等1秒再试，防止狂刷终端
                    continue

            # 2. 正常读取与解析逻辑 (保留你原本的代码，外加一层硬件异常捕获)
            try:
                if self.ser.in_waiting > 0:
                    # 加上 errors='ignore' 防止拔线瞬间产生乱码报错
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if not line: 
                        continue
                        
                    # 你的原始解析逻辑
                    try:
                        cmd_data = json.loads(line)
                        self.process_command(cmd_data)  # 直接交给你专门的函数处理！
                    except json.JSONDecodeError:
                        pass
                    except Exception as e:
                        rospy.logwarn(f"指令执行异常: {e}")

            # 3. 核心修复：捕获拔线导致的底层硬件异常
            except (OSError, serial.SerialException) as e:
                rospy.logerr("【硬件异常】数据线被拔出或通信中断！正在进入重连模式...")
                # 安全关闭损坏的句柄并置空，触发下一轮的自动重连
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
                rospy.sleep(1.0)
                
            except Exception as e:
                rospy.logerr(f"未知串口读取异常: {e}")
                rospy.sleep(0.5)

    # ================= 核心指令解析与执行逻辑 =================
    def process_command(self, data):
        cmd_type = data.get("type")
        
        if cmd_type == "CONNECT":
            rospy.loginfo("\n" + "="*45 + "\n  上位机 (Windows) 连接成功！通信链路已激活 \n" + "="*45)
            return
            
        rospy.loginfo(f"收到上位机指令: {cmd_type}")

        if cmd_type == "ARM":
            try:
                self.arm_service(value=data["value"])
            except rospy.ServiceException as e:
                rospy.logwarn(f"解锁/上锁服务调用失败: {e}")

        elif cmd_type == "SET_MODE":
            try:
                self.mode_service(base_mode=0, custom_mode=data["mode"])
            except rospy.ServiceException as e:
                rospy.logwarn(f"切换模式失败: {e}")

        elif cmd_type == "SET_HOME":
            self.execute_set_home()

        elif cmd_type == "NAV":
            self.cmd_pub.publish(String(data=json.dumps({"type": "NAV", "target": data["target"]})))

        elif cmd_type == "CHARGE":
            self.cmd_pub.publish(String(data=json.dumps({"type": "CHARGE"})))

        elif cmd_type == "TOGGLE_PATROL":
            self.patrol_pub.publish(Bool(data=data["value"]))

        elif cmd_type == "TOGGLE_AUTOCHARGE":
            self.autocharge_pub.publish(Bool(data=data["value"]))
            rospy.loginfo(f"已将自动充电使能状态设为: {data['value']}")

        elif cmd_type == "STOP_TASK":
            self.execute_stop_task()

    # ================= 具体动作执行函数 =================
    def execute_set_home(self):
        geo_msg = GeoPointStamped()
        geo_msg.header.stamp = rospy.Time.now()
        geo_msg.header.frame_id = 'map'
        geo_msg.position.latitude = 0
        geo_msg.position.longitude = 0
        geo_msg.position.altitude = 0
        self.origin_pub.publish(geo_msg)
        rospy.loginfo("已发布 EKF 原点 (0,0,0)")

        try:
            resp = self.home_service(current_gps=False, yaw=0, 
                                     latitude=FIXED_HOME_LAT, 
                                     longitude=FIXED_HOME_LON, 
                                     altitude=FIXED_HOME_ALT)
            if resp.success:
                rospy.loginfo("HOME 坐标设置成功")
            else:
                rospy.logwarn("HOME 坐标设置被拒绝")
        except rospy.ServiceException as e:
            rospy.logwarn(f"设置 HOME 坐标失败: {e}")

    def execute_stop_task(self):
        """处理紧急停止任务：网关只传话，刹车交由行为树执行"""
        rospy.logwarn("执行紧急停止任务 (交由行为树接管)")
        self.cmd_pub.publish(String(data=json.dumps({"type": "STOP"})))

if __name__ == '__main__':
    bridge = SerialBridge()
    rospy.spin()