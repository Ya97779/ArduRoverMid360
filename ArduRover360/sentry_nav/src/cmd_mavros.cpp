#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/PositionTarget.h>

class CmdVelToMavrosSerial
{
public:
    CmdVelToMavrosSerial()
        : nh_(), pnh_("~"),
          target_vx_(0.0), target_wz_(0.0), 
          is_braking_(false) // 增加一个状态标志位
    {
        // ===== 参数配置 =====
        pnh_.param<std::string>("input_topic", input_topic_, "/cmd_vel");
        // 注意：ArduPilot Rover 必须发往这个话题
        pnh_.param<std::string>("output_topic", output_topic_, "/mavros/setpoint_raw/local");
        
        pnh_.param<double>("max_linear_x", max_linear_x_, 2.0);
        pnh_.param<double>("max_angular_z", max_angular_z_, 2.0);
        
        // 【关键配置】
        // 串口直连虽然延迟低，但为了安全，如果超过 0.3秒 没收到新指令，就认为上层挂了，必须刹车。
        pnh_.param<double>("timeout", cmd_timeout_, 0.3);
        // 发送频率 20Hz (50ms)，ArduPilot 推荐 > 2Hz
        pnh_.param<double>("publish_rate", publish_rate_, 20.0);

        // ===== 订阅者 =====
        // tcpNoDelay 在串口模式下无所谓，但保留好习惯
        sub_ = nh_.subscribe<geometry_msgs::Twist>(
            input_topic_, 1, &CmdVelToMavrosSerial::cmdVelCallback, this, ros::TransportHints().tcpNoDelay());

        // ===== 发布者 =====
        pub_ = nh_.advertise<mavros_msgs::PositionTarget>(output_topic_, 10);

        // ===== 定时器 =====
        double period = (publish_rate_ > 0) ? (1.0 / publish_rate_) : 0.05;
        timer_ = nh_.createTimer(ros::Duration(period), &CmdVelToMavrosSerial::timerCallback, this);

        // 初始化时间，防止刚启动就误判超时
        last_cmd_time_ = ros::WallTime::now();

        // 掩码设置：
        // FRAME_BODY_NED (8): x向前, y向右, z向下
        // IGNORE_YAW (1024): 使用 yaw_rate 控制
        // FORCE (512): 强制执行
        type_mask_ =
            mavros_msgs::PositionTarget::IGNORE_PX |
            mavros_msgs::PositionTarget::IGNORE_PY |
            mavros_msgs::PositionTarget::IGNORE_PZ |
            mavros_msgs::PositionTarget::IGNORE_AFX |
            mavros_msgs::PositionTarget::IGNORE_AFY |
            mavros_msgs::PositionTarget::IGNORE_AFZ |
            mavros_msgs::PositionTarget::FORCE |
            mavros_msgs::PositionTarget::IGNORE_YAW;

        ROS_INFO("Mavros Serial Control Node Started.");
        ROS_INFO("TimeOut Threshold: %.2f s (Active Braking Enabled)", cmd_timeout_);
    }

private:
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {
        last_cmd_time_ = ros::WallTime::now(); // 喂狗：更新时间

        double vx = msg->linear.x;
        double wz = msg->angular.z;

        // 简单的限幅
        if (vx > max_linear_x_) vx = max_linear_x_;
        if (vx < -max_linear_x_) vx = -max_linear_x_;
        if (wz > max_angular_z_) wz = max_angular_z_;
        if (wz < -max_angular_z_) wz = -max_angular_z_;

        target_vx_ = vx;
        target_wz_ = wz;
        is_braking_ = false; // 收到新指令，解除刹车状态
    }

    void timerCallback(const ros::TimerEvent&)
    {
        // 1. 计算距离上一次收到 cmd_vel 过去了多久
        double dt = (ros::WallTime::now() - last_cmd_time_).toSec();

        // 2. 超时判断逻辑
        if (dt > cmd_timeout_) {
            target_vx_ = 0.0;
            target_wz_ = 0.0;
            
            // 为了防止刷屏日志，只在刚开始超时的时候打印一次
            if (!is_braking_) {
                ROS_WARN("Cmd_vel timeout (%.2fs)! Sending ZERO velocity to FCU.", dt);
                is_braking_ = true; 
            }
        }

        // 3. 构建并发送 MAVROS 消息
        mavros_msgs::PositionTarget sp;
        sp.header.stamp = ros::Time::now();
        sp.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
        sp.type_mask = type_mask_;

        sp.velocity.x = target_vx_;
        sp.velocity.y = 0.0;
        sp.velocity.z = 0.0;
        sp.yaw_rate   = target_wz_;

        pub_.publish(sp);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    ros::Timer timer_;

    std::string input_topic_;
    std::string output_topic_;
    double max_linear_x_, max_angular_z_;
    double cmd_timeout_, publish_rate_;
    uint16_t type_mask_;

    // 状态变量
    double target_vx_;
    double target_wz_;
    bool is_braking_;
    ros::WallTime last_cmd_time_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cmd_mavros_node");
    CmdVelToMavrosSerial node;
    ros::spin();
    return 0;
}