#include <velocity_smoother_ema/velocity_smoother_ema.hpp>

VelocitySmootherEma::VelocitySmootherEma(ros::NodeHandle* nh)
    : nh_(*nh)
{
    // 使用私有参数（建议 launch 里用 <node ...> 下的 <param>）
    nh_.param<std::string>("raw_cmd_topic", raw_cmd_topic_, std::string("raw_cmd_vel"));
    nh_.param<std::string>("cmd_topic", cmd_topic_, std::string("cmd_vel"));
    nh_.param<double>("alpha_v", alpha_v_, 0.4);
    nh_.param<double>("alpha_w", alpha_w_, 0.4);
    nh_.param<double>("cmd_rate", cmd_rate_, 30.0);
    nh_.param<double>("timeout", timeout_, 0.2);

    // clamp 一下 alpha，避免配错
    if (alpha_v_ < 0.0) alpha_v_ = 0.0;
    if (alpha_v_ > 1.0) alpha_v_ = 1.0;
    if (alpha_w_ < 0.0) alpha_w_ = 0.0;
    if (alpha_w_ > 1.0) alpha_w_ = 1.0;

    // 初始化：默认输入为 0
    last_cmd_ = geometry_msgs::Twist();
    last_msg_time_ = ros::WallTime::now();

    // Subscriber：queue=1 防堆积；tcpNoDelay 降延迟
    velocity_sub_ = nh_.subscribe<geometry_msgs::Twist>(
        raw_cmd_topic_, 1,
        &VelocitySmootherEma::twist_callback, this,
        ros::TransportHints().tcpNoDelay()
    );

    // Publisher：不要 latch，避免下游重连拿到旧速度
    velocity_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_topic_, 10, false);

    // Timer：周期发布平滑后的 cmd_vel
    const double period = (cmd_rate_ > 1e-6) ? (1.0 / cmd_rate_) : 0.0333;
    timer_ = nh_.createTimer(ros::Duration(period), &VelocitySmootherEma::update, this);

    ROS_INFO_STREAM("[velocity_smoother_ema] started"
                    << " raw_cmd_topic=" << raw_cmd_topic_
                    << " cmd_topic=" << cmd_topic_
                    << " alpha_v=" << alpha_v_
                    << " alpha_w=" << alpha_w_
                    << " cmd_rate=" << cmd_rate_
                    << " timeout=" << timeout_);
}

void VelocitySmootherEma::twist_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(mtx_);
    last_cmd_ = *msg;
    last_msg_time_ = ros::WallTime::now();
}

void VelocitySmootherEma::update(const ros::TimerEvent&)
{
    geometry_msgs::Twist in;
    double dt = 0.0;

    {
        std::lock_guard<std::mutex> lock(mtx_);
        in = last_cmd_;
        dt = (ros::WallTime::now() - last_msg_time_).toSec();
    }

    // 超时：把输入当作 0（立即开始向 0 收敛）
    // 如果你希望“超时就立刻输出绝对 0（不做平滑）”，我在下面也给了写法
    if (dt > timeout_) {
        in = geometry_msgs::Twist();
    }

    // EMA 平滑
    const double x = in.linear.x;
    const double y = in.linear.y;
    const double w = in.angular.z;

    const double sx = alpha_v_ * x + (1.0 - alpha_v_) * prev_x_;
    const double sy = alpha_v_ * y + (1.0 - alpha_v_) * prev_y_;
    const double sw = alpha_w_ * w + (1.0 - alpha_w_) * prev_w_;

    prev_x_ = sx;
    prev_y_ = sy;
    prev_w_ = sw;

    geometry_msgs::Twist out;
    out.linear.x  = sx;
    out.linear.y  = sy;
    out.angular.z = sw;

    //——如果你想“超时后立刻输出 0（不做EMA缓慢衰减）”，改成下面这样：
    if (dt > timeout_) {
        prev_x_ = prev_y_ = prev_w_ = 0.0;
        out = geometry_msgs::Twist();
    }

    velocity_pub_.publish(out);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "velocity_smoother_ema");
    ros::NodeHandle nh("~");   // 私有参数空间
    VelocitySmootherEma node(&nh);
    ros::spin();
    return 0;
}
