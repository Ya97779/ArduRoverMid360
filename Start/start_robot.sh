#!/bin/bash

echo "====================================================="
echo "       🚀 ArduRover 顺序启动与系统自检序列           "
echo "====================================================="

# ================= 环境配置 =================
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
# ==========================================

# 杀掉残留的 tmux 会话
tmux kill-session -t rover 2>/dev/null
sleep 1
tmux new-session -d -s rover

# ---------------------------------------------------------
# 核心函数：在 tmux 中启动并在主终端进行存活检测
# 参数：$1=窗口名, $2=启动命令, $3=中文描述, $4=进程检测关键词, $5=等待时间(秒)
# ---------------------------------------------------------
launch_and_check() {
    local win_name=$1
    local cmd=$2
    local desc=$3
    local keyword=$4
    local wait_sec=$5

    echo -ne "⏳ 正在启动: $desc ... "
    
    # 新建 tmux 窗口并注入死循环守护命令
    tmux new-window -t rover -n "$win_name"
    tmux send-keys -t rover:"$win_name" "while true; do $cmd; sleep 2; done" C-m
    
    # 给出程序初始化的时间
    sleep "$wait_sec"
    
    # 扫描系统进程表，看关键词是否存在
    if pgrep -f "$keyword" > /dev/null; then
        echo -e "\e[32m[✅ 启动成功]\e[0m"
    else
        echo -e "\e[31m[❌ 启动异常]\e[0m (请稍后进入 tmux a -t rover:$win_name 查看日志)"
    fi
}

# ---------------------------------------------------------
# Step 0: 启动底座 roscore
# ---------------------------------------------------------
echo -ne "⏳ 正在启动:Roscore ... "
tmux rename-window -t rover:0 'roscore'
tmux send-keys -t rover:0 'while true; do roscore; sleep 2; done' C-m
sleep 3
if pgrep -f "roscore" > /dev/null; then
    echo -e "\e[32m[✅ 启动成功]\e[0m"
else
    echo -e "\e[31m[❌ 启动异常]\e[0m"
fi
echo "-----------------------------------------------------"

# ---------------------------------------------------------
# 依次启动用户定义的各项服务 (严格按顺序)
# ---------------------------------------------------------

# 1. 启动 Mavros
launch_and_check "mavros" "sh ~/mavros.sh" "Mavros 连接飞控" "mavros" 5

# ---------------------------------------------------------
# 单次任务：设置 EKF 原点与 Home 点
# ---------------------------------------------------------
echo -ne "📍 正在下发: 设置 EKF 原点与 Home 点 ... "
tmux new-window -t rover -n 'ekf_home'
# 这里的逻辑是：执行脚本 -> 打印提示 -> 退出并关闭当前 tmux 窗口 (不用死循环)
tmux send-keys -t rover:'ekf_home' 'sh ~/set_ekf_home.sh; echo "EKF 设置完毕，退出窗口"; sleep 2; exit' C-m
sleep 5 # 给脚本执行留出 5 秒钟的时间
echo -e "\e[32m[✅ 下发完毕]\e[0m"

# 2. 启动雷达
launch_and_check "lidar" "roslaunch livox_ros_driver2 msg_MID360.launch" "MID360 雷达驱动" "msg_MID360" 4

# 3. 启动重定位
launch_and_check "fast_lio" "roslaunch fast_lio_localization sentry_localize.launch" "FAST_LIO2 重定位" "sentry_localize" 5

# ---------------------------------------------------------
# 插入自动发布初始位姿的任务 (针对重定位)
# ---------------------------------------------------------
echo -ne "📍 正在下发: 初始位姿 (2D Pose) ... "
tmux new-window -t rover -n 'init_pose'
tmux send-keys -t rover:'init_pose' 'for i in 1 2 3; do rostopic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped "{header: {frame_id: '\''map'\''}, pose: {pose: {position: {x: -1.113, y: -0.112, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.99988, w: 0.01537}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0685]}}"; sleep 2; done; exit' C-m
sleep 7 # 给它 7 秒钟把 3 次坐标发完
echo -e "\e[32m[✅ 下发完毕]\e[0m"

# 4. 启动导航
launch_and_check "move_base" "roslaunch sentry_nav sentry_movebase.launch" "导航避障" "sentry_movebase" 5

# 6. 启动充电监控GPIO
launch_and_check "charge_monitor" "sh ~/charging_state.sh" "充电状态监控" "charging_state" 3

# 7. 启动对接算法
launch_and_check "autodock" "roslaunch autodock_sim rover_sim.launch" "AutoDock 自动对接" "autodock_sim" 4

# 8. 启动串口bridge 
launch_and_check "serial_bridge" "python3 ~/catkin_ws/src/behavior_tree/serial_bridge_node_new.py" "上位机串口桥接网关" "serial_bridge_node" 3

# 9. 启动电池状态mavlink自定义数据监控
launch_and_check "mav240" "python3 ~/catkin_ws/src/behavior_tree/mavlinkmsg240.py" "电池状态监控节点" "mavlinkmsg240" 6

# 10.启动行为树
launch_and_check "behavior_tree" "python3 ~/catkin_ws/src/behavior_tree/bhtree_final.py" "行为树(Behavior Tree)" "bhtree_final" 3

echo "====================================================="
echo -e "\e[32m🎉 所有节点均已按顺序排队拉起并完成自检！\e[0m"
echo "提示: 如有异常，可输入 tmux a -t rover 进入后台查看具体日志。"
echo "====================================================="