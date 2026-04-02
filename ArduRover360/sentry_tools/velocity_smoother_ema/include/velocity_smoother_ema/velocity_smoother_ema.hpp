#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <mutex>

class VelocitySmootherEma
{
public:
    explicit VelocitySmootherEma(ros::NodeHandle* nh);
    ~VelocitySmootherEma() = default;

private:
    void twist_callback(const geometry_msgs::Twist::ConstPtr& msg);
    void update(const ros::TimerEvent&);

private:
    ros::NodeHandle nh_;
    ros::Subscriber velocity_sub_;
    ros::Publisher velocity_pub_;
    ros::Timer timer_;

    std::string raw_cmd_topic_;
    std::string cmd_topic_;

    double alpha_v_{0.4};
    double alpha_w_{0.4};
    double cmd_rate_{30.0};
    double timeout_{0.2};

    // 最新输入指令（raw）
    geometry_msgs::Twist last_cmd_;
    ros::WallTime last_msg_time_;

    // EMA 状态
    double prev_x_{0.0}, prev_y_{0.0}, prev_w_{0.0};

    std::mutex mtx_;
};
