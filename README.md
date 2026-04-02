# 实现的功能：建图、重定位、导航、避障、自动充电
视频演示：【MID360+单目实现差速小车重定位、导航避障与自动充电】 https://www.bilibili.com/video/BV12gXhBhEtp/?share_source=copy_web&vd_source=1d13dd5ae897228eaa8f3bbc38f8aba6
# 🚀 ArduRover-Mid360: 移动机器人系统

![ROS](https://img.shields.io/badge/ROS-Noetic-brightgreen)
![Hardware](https://img.shields.io/badge/Hardware-Jetson_Orin_NX-blue)
![LiDAR](https://img.shields.io/badge/Sensor-Livox_Mid--360-orange)
![License](https://img.shields.io/badge/License-MIT-green)

本项目是一个基于 **NVIDIA Jetson Orin NX** 算力平台与 **Livox Mid-360** 激光雷达构建的全栈式自主移动机器人底盘系统。项目涵盖了从底层飞控通信（MAVLink）、多传感器融合 SLAM、自主导航规划、高层行为树（Behavior Tree）决策，到跨平台跨端 GUI 监控的完整软硬件闭环。

## 🌟 核心功能与特色 (Key Features)

- **高精度定位与建图**：部署了 `FAST-LIO2` 激光惯导里程计算法，并结合 `FastLioLocalization` 实现了高鲁棒性的全局重定位。
- **自主导航与避障**：基于 `MoveBase` 框架，结合 `TEB Local Planner` 实现了考虑车辆运动学约束的平滑路径规划与动态避障。
- **智能决策树 (Behavior Tree)**：弃用传统状态机，采用基于 `py_trees_ros` 的 Python 决策树脚本。实现了极其灵活的任务编排、优先级抢占。
- **串口通信**：开发跨平台串口通信桥（Serial Bridge），采用 JSON 协议。内置了断线检测与“热插拔自动重连”机制，提高底盘与 Windows 上位机之间的通信稳定性。
- **自定义飞控消息解析**：实现对 ArduPilot (Rover) 飞控 MAVLink 协议的深度解析，解析并转发自定义mavlink消息包中的电池数据。
- **视觉对接与自动回充**：集成 USB 摄像头与 AprilTag 识别算法实现自动对桩充电。
- **跨平台 GUI 监控站**：使用 Python `tkinter` 库开发了 Windows 端的上位机用户控制与实时监测界面，实现对底层设备人工调度。

## 🏗️ 硬件架构 (Hardware Stack)

- **核心计算平台**: NVIDIA Jetson Orin NX
- **主传感器**: Livox Mid-360 (3D 固态激光雷达)
- **视觉传感器**: 高清 USB 免驱摄像头 (针对自动对接优化)
- **底层控制器**: ArduPilot Rover 飞控

## 📂 核心项目结构 (Repository Structure)

```text
ArduRoverMid360/
├── sentry_nav/            # 导航层：MoveBase 配置及 TEB 局部路径规划器调参
├── sentry_slam/           # 建图层：FAST-LIO2 及 FastLioLocalization 部署与启动配置
└── sentry_tools/           # 工具层：包含3d点云pcd文件转化二位栅格地图和用于平滑输出控制命令的ros包

behavior_tree/
├── bhtree_final/          # 行为树核心决策层：基于 py_trees_ros 的行为树逻辑与自定义节点
├── charging_state/        # 充电状态监控层：读取NX的GPIO端口用于判断是否在充电
├── mavlinkmsg240/         # 自定义mavlink消息包解析：自定义 MAVLink 报文解包器 (获取电池数据)
└── serial_bridge_node_new # 通信层：Windows 上位机与 ROS 系统的 JSON 串口双向网关

autodock/                  # 自动对齐充电桩ROS包

livox_ros_driver2/         # Mid360激光雷达官方驱动
```
## 感谢：(Thanks to)

- 定位与导航部分借鉴于开源仓库：https://github.com/66Lau/NEXTE_Sentry_Nav.git
- 对齐充电桩部分借鉴于开源仓库：https://github.com/osrf/autodock 

