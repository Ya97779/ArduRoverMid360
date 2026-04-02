🌐 **English Version** | [简体中文](README.md)
# Features Implemented: Mapping, Relocation, Navigation, Obstacle Avoidance, Automatic Charging
Video demonstration: 【MID360+ Monocular Differential Mobile Robot Relocation, Navigation, Obstacle Avoidance, and Automatic Charging】 https://www.bilibili.com/video/BV12gXhBhEtp/?share_source=copy_web&vd_source=1d13dd5ae897228eaa8f3bbc38f8aba6
# 🚀 ArduRover-Mid360: Mobile Robot System

![ROS](https://img.shields.io/badge/ROS-Noetic-brightgreen)
![Hardware](https://img.shields.io/badge/Hardware-Jetson_Orin_NX-blue)
![LiDAR](https://img.shields.io/badge/Sensor-Livox_Mid--360-orange)
![License](https://img.shields.io/badge/License-MIT-green)

This project is a full-stack autonomous mobile robot chassis system built on the **APM flight control**, **NVIDIA Jetson Orin NX** computing platform, and **Livox Mid-360** LiDAR. The project encompasses a complete hardware and software loop, from low-level flight control communication (MAVLink), multi-sensor fusion SLAM, autonomous navigation planning, high-level behavior tree decision-making, to cross-platform GUI monitoring.

## 🌟 Key Features

- **High-Precision Localization and Mapping**: Deployed the `FAST-LIO2` laser inertial odometry algorithm, combined with `FastLioLocalization` to achieve robust global relocation.
- **Autonomous Navigation and Obstacle Avoidance**: Based on the `MoveBase` framework, combined with `TEB Local Planner` to achieve smooth path planning and dynamic obstacle avoidance considering vehicle kinematic constraints.
- **Intelligent Decision Tree**: Replaced traditional state machines with Python decision tree scripts based on `py_trees_ros`. This allows for extremely flexible task orchestration and priority preemption.
- **Serial Communication**: Developed a cross-platform serial communication bridge (Serial Bridge) using JSON protocol. It includes disconnection detection and a "hot-plug automatic reconnection" mechanism to enhance communication stability between the chassis and Windows host.
- **Custom Flight Control Message Parsing**: Implemented in-depth parsing of ArduPilot (Rover) flight control MAVLink protocol, parsing and forwarding battery data from custom MAVLink message packets.
- **Visual Docking and Automatic Recharging**: Integrated USB camera and AprilTag recognition algorithm for automatic docking and charging.
- **Cross-Platform GUI Monitoring Station**: Developed a user control and real-time monitoring interface for the Windows host using Python `tkinter`, enabling manual scheduling of underlying devices.

## 🏗️ Hardware Stack

- **Core Computing Platform**: NVIDIA Jetson Orin NX
- **Main Sensor**: Livox Mid-360 (3D Solid-State LiDAR)
- **Visual Sensor**: High-definition USB plug-and-play camera (optimized for automatic docking)
- **Low-Level Controller**: ArduPilot Rover flight control

## Rviz
<img width="2560" height="1349" alt="image" src="https://github.com/user-attachments/assets/5f307a5f-97c3-416b-93c7-ba5e9dfe542b" />
<img width="2314" height="1319" alt="image" src="https://github.com/user-attachments/assets/8c4ad7e1-c770-4244-ad86-cca90706db0e" />

## GUI Interface
<img width="626" height="1234" alt="image" src="https://github.com/user-attachments/assets/701132d9-8de7-4a1e-8232-a932fb0c268b" />



## 📂 Repository Structure

```text
src
└──ArduRoverMid360/
|      ├── sentry_nav/             # Navigation layer: MoveBase configuration and TEB local planner parameter tuning
|      ├── sentry_slam/           # Mapping layer: FAST-LIO2 and FastLioLocalization deployment and launch configs
|      └── sentry_tools/            # Tool layer: Contains ROS packages for converting 3D point cloud PCD files to 2D grid maps and smoothing output control commands
|
└──behavior_tree/
|     ├── bhtree_final/          # Behavior Tree core decision layer: py_trees_ros based logic and custom nodes
|     ├── charging_state/        # Charging state monitoring layer: Reads NX GPIO ports to determine charging status
|     ├── mavlinkmsg240/         # Custom MAVLink message parsing: Custom MAVLink packet unpacker (to get battery data)
|     └── serial_bridge_node_new # Communication layer: JSON serial bidirectional gateway between Windows host and ROS system
|
└──autodock/                  # Auto-docking charging station ROS package
|
└──livox_ros_driver2/         # Official Livox Mid-360 LiDAR driver
|
└──Start/                     # tmux one-click start script and host GUI interface
     ├── winGUI/               # User interface
     └── start_robot.sh/      # One-click start tool

```

## Launch Steps: You can refer to the tmux session script written in Start/start_robot.sh
### Method 1: Start everything at once using a bash script:
-  ```git clone https://github.com/Ya97779/ArduRoverMid360.git```
-  ```cd ~/catkin_ws```
-  ```catkin_make```
-  ```source devel/setup.bash```
-  ```bash ~/catkin_ws/src/Start/start_robot.sh``` (You need to install tmux in the terminal and modify the specific path of the .sh file as needed)
### Method 2: Start individually
- 1. Start mavros to connect to the flight controller:       ```roslaunch mavros apm.launch```
- 2. Set the flight controller EKF origin and Home: You can either click in the GUI or write your own publishing script
- 3. Start the radar driver:             ``` roslaunch livox_ros_driver2 msg_MID360.launch```
- 4. Start FastLio2 localization:       ``` roslaunch fast_lio_localization sentry_localize.launch```
- 5. Publish the initial pose: You can use rviz or write your own script to publish to the initial pose topic
- 6. Start the navigation and obstacle avoidance stack:           ``` roslaunch sentry_nav sentry_movebase.launch```
- 7. Start GPIO charging monitoring:         ``` python3 ~/catkin_ws/src/behavior_tree/charging_state.py```
- 8. Start AutoDock:             ``` roslaunch autodock_sim rover_sim.launch```
- 9. Start the serial communication gateway:         ``` python3 ~/catkin_ws/src/behavior_tree/serial_bridge_node_new.py```
- 10. Start the battery data monitoring node:    ``` python3 ~/catkin_ws/src/behavior_tree/mavlinkmsg240.py```
- 11. Start the behavior tree:              ``` python3 ~/catkin_ws/src/behavior_tree/bhtree_final.py```
- 12. Start the user GUI interface:          ``` python3 ~/catkin_ws/src/Start/winGUI.py```


## Thanks to

- The localization and navigation part was developed based on the open-source repository: https://github.com/66Lau/NEXTE_Sentry_Nav.git
- The alignment of the charging station was developed based on the open-source repository: https://github.com/osrf/autodock


