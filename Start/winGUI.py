import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import json
import threading
import time

class WindowsCommander:
    def __init__(self, master):
        self.master = master
        master.title("ArduRover — 控制台")
        master.geometry("500x950")

        # --- 串口底层变量 ---
        self.ser = None
        self.is_serial_open = False
        self.rx_thread_running = True
        
        # 业务状态变量
        self.patrol_enabled = False
        self.autocharge_enabled = False

        # [修改] 电池警告标志位，按百分比防抖
        self.warned_30_pct = False
        self.warned_15_pct = False

        # 提前开启后台接收线程（它会一直循环等待串口打开）
        threading.Thread(target=self.rx_thread, daemon=True).start()

        # --- 界面布局 ---
        tk.Label(master, text="移动底盘控制台", font=("sans-serif", 20, "bold")).pack(pady=10)

        # ================= 串口通信设置区 =================
        self.frame_serial = tk.LabelFrame(master, text="[通信设置]", padx=10, pady=5, fg="black", font=("sans-serif", 10, "bold"))
        self.frame_serial.pack(fill="x", padx=10, pady=5)

        tk.Label(self.frame_serial, text="端口:").grid(row=0, column=0, padx=5, pady=5)
        self.cmb_ports = ttk.Combobox(self.frame_serial, width=10)
        self.cmb_ports.grid(row=0, column=1, padx=5, pady=5)
        
        self.btn_refresh = tk.Button(self.frame_serial, text="刷新", command=self.refresh_ports)
        self.btn_refresh.grid(row=0, column=2, padx=5, pady=5)

        tk.Label(self.frame_serial, text="波特率:").grid(row=0, column=3, padx=5, pady=5)
        self.cmb_baud = ttk.Combobox(self.frame_serial, values=["9600", "57600", "115200", "921600"], width=10, state="readonly")
        self.cmb_baud.set("115200")
        self.cmb_baud.grid(row=0, column=4, padx=5, pady=5)

        self.btn_open_serial = tk.Button(self.frame_serial, text="打开串口", bg="#4caf50", fg="white", font=("sans-serif", 10, "bold"), command=self.toggle_serial)
        self.btn_open_serial.grid(row=0, column=5, padx=10, pady=5)

        # ================= 状态监控区 =================
        self.frame_status = tk.LabelFrame(master, text="[实时状态监控]", padx=10, pady=10, fg="green", font=("sans-serif", 10, "bold"))
        self.frame_status.pack(fill="x", padx=10, pady=5)

        self.lbl_pose = tk.Label(self.frame_status, text="机器人坐标: X = 0.00, Y = 0.00", font=("sans-serif", 12))
        self.lbl_pose.pack(anchor="w", pady=2)
        
        self.lbl_task = tk.Label(self.frame_status, text="当前任务: 等待连接...", font=("sans-serif", 12, "bold"), fg="blue")
        self.lbl_task.pack(anchor="w", pady=2)

        # [修改] 电池 UI 更新为显示全面信息
        self.lbl_battery = tk.Label(self.frame_status, text="电池状态: --% | --V | --A", font=("sans-serif", 12))
        self.lbl_battery.pack(anchor="w", pady=2)

        # ================= 飞控基础指令区 =================
        self.frame_apm = tk.LabelFrame(master, text="[Step 1] 基础控制", padx=10, pady=10, fg="blue", font=("sans-serif", 10, "bold"))
        self.frame_apm.pack(fill="x", padx=10, pady=5)

        frame_power = tk.Frame(self.frame_apm)
        frame_power.pack(side=tk.TOP, fill="x", pady=5)
        tk.Button(frame_power, text="解锁 (ARM)", bg="#69f0ae", width=18, height=2, font=("sans-serif", 10, "bold"),
                  command=lambda: self.send_cmd({"type": "ARM", "value": True})).pack(side=tk.LEFT, padx=5)
        tk.Button(frame_power, text="上锁 (DISARM)", bg="#ff8a80", width=18, height=2, font=("sans-serif", 10, "bold"),
                  command=lambda: self.send_cmd({"type": "ARM", "value": False})).pack(side=tk.RIGHT, padx=5)

        frame_modes = tk.Frame(self.frame_apm)
        frame_modes.pack(side=tk.TOP, fill="x", pady=5)
        tk.Button(frame_modes, text="MANUAL (手动)", bg="#cfd8dc", width=18,
                  command=lambda: self.send_cmd({"type": "SET_MODE", "mode": "MANUAL"})).pack(side=tk.LEFT, padx=5)
        tk.Button(frame_modes, text="GUIDED (自动)", bg="orange", width=18,
                  command=lambda: self.send_cmd({"type": "SET_MODE", "mode": "GUIDED"})).pack(side=tk.RIGHT, padx=5)

        tk.Button(self.frame_apm, text="设置 EKF Origin 和 HOME", bg="#b39ddb", height=2,
                  command=lambda: self.send_cmd({"type": "SET_HOME"})).pack(fill="x", padx=5, pady=5)

        # ================= 导航与调度任务区 =================
        self.frame_nav = tk.LabelFrame(master, text="[Step 2] 导航与行为调度", padx=10, pady=10, fg="purple", font=("sans-serif", 10, "bold"))
        self.frame_nav.pack(fill="x", padx=10, pady=5)

        self.btn_stop = tk.Button(self.frame_nav, text="  取消当前任务 ", bg="#d50000", fg="white", font=("sans-serif", 12, "bold"), height=2,
                                  command=lambda: self.send_cmd({"type": "STOP_TASK"}))
        self.btn_stop.pack(fill="x", padx=5, pady=5)

        frame_nav_top = tk.Frame(self.frame_nav)
        frame_nav_top.pack(side=tk.TOP, fill="x", pady=5)
        tk.Button(frame_nav_top, text="去工位", bg="#e040fb", fg="white", height=2,
                  command=lambda: self.send_cmd({"type": "NAV", "target": "POSE1"})).pack(side=tk.LEFT, padx=2, expand=True, fill="x")
        tk.Button(frame_nav_top, text="去门口", bg="#aa00ff", fg="white", height=2,
                  command=lambda: self.send_cmd({"type": "NAV", "target": "POSE2"})).pack(side=tk.LEFT, padx=2, expand=True, fill="x")
        tk.Button(frame_nav_top, text="去会议桌", bg="#7200ca", fg="white", height=2,
                  command=lambda: self.send_cmd({"type": "NAV", "target": "POSE3"})).pack(side=tk.LEFT, padx=2, expand=True, fill="x")

        tk.Button(self.frame_nav, text="去充电 (单次强制)", bg="#f57c00", fg="white", height=2, font=("sans-serif", 10, "bold"),
                  command=lambda: self.send_cmd({"type": "CHARGE"})).pack(fill="x", padx=5, pady=5)

        self.btn_autocharge = tk.Button(self.frame_nav, text="自动充电: 已关闭", bg="#ff6d00", fg="white", height=2, font=("sans-serif", 10, "bold"),
                                        command=self.toggle_autocharge)
        self.btn_autocharge.pack(fill="x", padx=5, pady=5)

        self.btn_patrol = tk.Button(self.frame_nav, text="自动巡航：已关闭", bg="#ff6d00", fg="white", height=2, font=("sans-serif", 10, "bold"),
                                    command=self.toggle_patrol)
        self.btn_patrol.pack(fill="x", padx=5, pady=5)

        self.status_label = tk.Label(master, text="请在上方选择并打开串口", fg="grey", font=("sans-serif", 10))
        self.status_label.pack(side=tk.BOTTOM, pady=10)

        self.refresh_ports()

    # ================= 串口与后台线程逻辑 =================

    def refresh_ports(self):
        ports = serial.tools.list_ports.comports()
        port_list = [port.device for port in ports]
        self.cmb_ports['values'] = port_list
        if port_list:
            self.cmb_ports.set(port_list[0])
        else:
            self.cmb_ports.set('')

    def toggle_serial(self):
        if self.is_serial_open:
            self.is_serial_open = False
            if self.ser and self.ser.is_open:
                self.ser.close()
            self.btn_open_serial.config(text="打开串口", bg="#4caf50")
            self.status_label.config(text="串口已断开", fg="grey")
            self.cmb_ports.config(state="normal")
            self.cmb_baud.config(state="readonly")
            self.btn_refresh.config(state="normal")
        else:
            port = self.cmb_ports.get()
            baud = self.cmb_baud.get()
            if not port:
                messagebox.showwarning("警告", "请先选择一个端口！")
                return
            try:
                self.ser = serial.Serial(port, int(baud), timeout=0.5)
                self.is_serial_open = True
                
                self.master.after(500, lambda: self.send_cmd({"type": "CONNECT"}))

                self.btn_open_serial.config(text="关闭串口", bg="#f44336")
                self.status_label.config(text=f"成功连接 {port} ({baud} bps)", fg="green")
                self.cmb_ports.config(state="disabled")
                self.cmb_baud.config(state="disabled")
                self.btn_refresh.config(state="disabled")
            except Exception as e:
                messagebox.showerror("连接失败", f"无法打开端口 {port}:\n{e}")

    def rx_thread(self):
        while self.rx_thread_running:
            if self.is_serial_open and self.ser and self.ser.is_open:
                try:
                    if self.ser.in_waiting > 0:
                        line = self.ser.readline().decode('utf-8').strip()
                        if line:
                            state = json.loads(line)
                            self.master.after(0, self.update_ui_state, state)
                except serial.SerialException:
                    self.master.after(0, self.handle_disconnect)
                except json.JSONDecodeError:
                    pass
                except Exception:
                    pass
            time.sleep(0.02)

    def handle_disconnect(self):
        if self.is_serial_open:
            self.toggle_serial()
            messagebox.showwarning("设备断开", "串口连接意外断开！\n请检查数据线是否松动，或者 Jetson 是否掉电。")

    def send_cmd(self, cmd_dict):
        if self.is_serial_open and self.ser and self.ser.is_open:
            try:
                msg = json.dumps(cmd_dict) + '\n'
                self.ser.write(msg.encode('utf-8'))
                
                action_name = cmd_dict.get('type', 'Unknown')
                if action_name == 'NAV':
                    action_name += f" ({cmd_dict.get('target')})"
                self.status_label.config(text=f"已发送指令: {action_name}", fg="blue")
            except Exception as e:
                self.status_label.config(text=f"发送失败: {e}", fg="red")
        else:
            messagebox.showwarning("提示", "请先在上方打开串口！")

    # ================= 业务界面逻辑 =================

    def update_ui_state(self, state):
        """[主线程] 解析 Jetson 发来的 JSON 并刷新界面"""
        # 1. 兼容原有状态 (如果有的话)
        if 'pose_x' in state and 'pose_y' in state:
            pose_x = state.get('pose_x', 0.0)
            pose_y = state.get('pose_y', 0.0)
            self.lbl_pose.config(text=f"坐标: X = {pose_x:.2f}, Y = {pose_y:.2f}")
        
        if 'task_status' in state:
            task_status = state.get('task_status', '未知')
            self.lbl_task.config(text=f"当前任务: {task_status}")
        
        # 2. [核心修改] 电池与弹窗逻辑
        # 兼容单独的 {"type": "BATT", "vol": x, "cur": x, "pct": x} 报文
        if state.get("type") == "BATT" or "pct" in state:
            vol = state.get('vol', 0.0)
            cur = state.get('cur', 0.0)
            pct = state.get('pct', 0.0)
            
            if pct > 0:
                # -- UI 状态显示逻辑 --
                if pct > 99 and cur < 1.3:
                    self.lbl_battery.config(text=f"电池状态: 已充满{pct:.0f}% | {vol:.1f}V | {cur:.1f}A", fg="#00c853", font=("sans-serif", 12, "bold"))
                elif pct < 30:
                    self.lbl_battery.config(text=f"电池状态: 电量低{pct:.0f}% | {vol:.1f}V | {cur:.1f}A  (电量低!)", fg="red", font=("sans-serif", 12, "bold"))
                else:
                    self.lbl_battery.config(text=f"电池状态: {pct:.0f}% | {vol:.1f}V | {cur:.1f}A", fg="black", font=("sans-serif", 12))

                # -- 业务逻辑：电量 < 15% 严重报警 --
                if pct < 15.0 and not self.warned_15_pct:
                    self.warned_15_pct = True
                    self.warned_30_pct = True # 防止 30% 弹窗重复弹出
                    self.show_critical_battery_dialog(pct, vol)
                    
                # -- 业务逻辑：< 30% 一般报警 --
                elif 15.0 <= pct < 30.0 and not self.warned_30_pct:
                    self.warned_30_pct = True
                    self.show_low_battery_dialog(pct, vol)
                    
                # -- 业务逻辑：恢复重置 (充到 35% 以上解开弹窗锁) --
                if pct >= 35.0:
                    self.warned_30_pct = False
                    self.warned_15_pct = False

    def toggle_patrol(self):
        self.patrol_enabled = not self.patrol_enabled
        self.send_cmd({"type": "TOGGLE_PATROL", "value": self.patrol_enabled})
        if self.patrol_enabled:
            self.btn_patrol.config(text="自动巡航：已开启", bg="#00c853")
        else:
            self.btn_patrol.config(text="自动巡航：已关闭", bg="#ff6d00")

    def toggle_autocharge(self):
        """[修改] 给 Jetson 发送是否允许自动充电的布尔值"""
        self.autocharge_enabled = not self.autocharge_enabled
        self.send_cmd({"type": "TOGGLE_AUTOCHARGE", "value": self.autocharge_enabled})
        if self.autocharge_enabled:
            self.btn_autocharge.config(text="自动充电: 允许", bg="#00c853")
        else:
            self.btn_autocharge.config(text="自动充电: 已关闭", bg="#ff6d00")

    def show_low_battery_dialog(self, pct, vol):
        """处理 < 30% 一般低电量提醒"""
        dialog = tk.Toplevel(self.master)
        dialog.title("普通警报 - 电量不足")
        dialog.geometry("320x150")
        dialog.attributes("-topmost", True)
        dialog.grab_set() 
        
        tk.Label(dialog, text=f"当前电量 {pct:.0f}% ({vol:.1f}V)\n建议开始返航充电。是否立即前往？", 
                 font=("sans-serif", 11, "bold"), fg="#ff8f00").pack(pady=20)
        
        btn_frame = tk.Frame(dialog)
        btn_frame.pack(fill="x", padx=10)
        
        def on_agree():
            self.send_cmd({"type": "CHARGE"})
            dialog.destroy()
            
        def on_decline():
            dialog.destroy()
            
        tk.Button(btn_frame, text="同意 (去充电)", bg="#4caf50", fg="white", font=("sans-serif", 10, "bold"),
                  command=on_agree).pack(side=tk.LEFT, expand=True, padx=5, ipady=5)
        tk.Button(btn_frame, text="暂不", bg="#9e9e9e", fg="white", font=("sans-serif", 10),
                  command=on_decline).pack(side=tk.RIGHT, expand=True, padx=5, ipady=5)

    def show_critical_battery_dialog(self, pct, vol):
        """处理 < 15% 极低电量报警"""
        dialog = tk.Toplevel(self.master)
        dialog.title(" 严重警报 - 极低电量")
        dialog.geometry("320x150")
        dialog.attributes("-topmost", True)
        dialog.grab_set()
        
        tk.Label(dialog, text=f"极低电量: {pct:.0f}% ({vol:.1f}V)！\n即将失去动力，请立即介入！", 
                 font=("sans-serif", 12, "bold"), fg="#d50000").pack(pady=20)
        
        btn_frame = tk.Frame(dialog)
        btn_frame.pack(fill="x", padx=10)
        
        def on_charge():
            self.send_cmd({"type": "CHARGE"})
            dialog.destroy()
            
        def on_pause():
            self.send_cmd({"type": "STOP_TASK"})
            dialog.destroy()
            
        tk.Button(btn_frame, text="强制去充电", bg="#f57c00", fg="white", font=("sans-serif", 10, "bold"),
                  command=on_charge).pack(side=tk.LEFT, expand=True, padx=5, ipady=5)
        tk.Button(btn_frame, text="紧急刹车", bg="#d50000", fg="white", font=("sans-serif", 10, "bold"),
                  command=on_pause).pack(side=tk.RIGHT, expand=True, padx=5, ipady=5)

if __name__ == "__main__":
    root = tk.Tk()
    app = WindowsCommander(root)
    def on_closing():
        app.rx_thread_running = False
        if app.ser and app.ser.is_open:
            app.ser.close()
        root.destroy()
    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()
