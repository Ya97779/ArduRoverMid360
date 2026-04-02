# 底盘导航系统安装调试步骤
## 软件
   1.对环境建图 ：①启动mid360驱动：roslaunch livox_ros_driver2 msg_MID360.launch
                ②启动slam建图：roslaunch fast_lio mapping_mid360.launch  
                ③遥控器控制小车走一圈进行建图，并保存为.pcd文件
                ④将PCD转化为2维栅格地图：roslaunch pcd2pgm run.launch  ，这会将转化后的地图发布到话题中去（位于sentry_tools/pcd2pgm_package/pcd2pgm/launch/run.launch）
                ⑤在PCD文件夹中保存转化后的2维栅格地图：rosrun map_server map_saver -f office_cpr_2d /map:=/map  ， office_cpr_2d为2维栅格地图的名字，

    2.重定位并运行里程计:roslaunch fast_lio_localization sentry_localize.launch，其中包含将里程计喂给飞控的odom_to_mavors_node 位于sentry_nav中

    3.运行move_base框架：roslaunch sentry_nav sentry_movebase.launch ，其中包含将/cmd_vel命令转化成/mavros/setpoint_raw/local的节点  位于sentry_nav中

    注：调飞控参数：修改飞控参数可以修改飞控的里程计源：EK3_SRC1_POSXY  EK3_SRC1_POSZ （默认1：Baro）  EK3_SRC1_VELXY  EK3_SRC1_VELZ 其余默认都是3：GPS  ，修改为6：外部里程计   EK3_SRC1_YAW 1->6     VISO_TYPE 0 -> 1：开启视觉里程计  SIM_GPS1_ENABLE  1 -> 0 ：关掉gps模拟    AHRS_GPS_USE = 0  不用gps做姿态/位置辅助  BRD_RTC_TYPES = 2  （时间同步）

0.设置系统时间
1.roslaunch livox_ros_driver2 msg_MID360.launch 
2.sh mavros.sh
3.sh set_ekf_home.sh 
4.roslaunch fast_lio_localization  sentry_localize.launch
5.rviz 点一下位置估计
6.roslaunch sentry_nav  sentry_movebase.launch 


注：飞控参数
1.通过cmd_vel命令控制小车原地转弯，停止命令后飞控PWM值恒有1500±x ，或者回的很慢，需要调整方向至舵机的PID，原因：I 项把里程计的微小偏置放大成稳定差速，把ATC_STR_RAT_I调到0即可
2.同样cmd_vel命令控制小车前后走，停止命令后飞控PWM回的很慢，首先要检查最大加速度ATC_ACCEL_MAX = 1和减速的最大加速度ATC_DECEL_MAX = 0，默认为0调整为3即可，之后根据需要调整速度至油门的PID。
3.注意rover下GUIDED模式是混合控制的，即可以接受遥控器的rc输入，因此遥控器都要回中以防影响导航效果。












