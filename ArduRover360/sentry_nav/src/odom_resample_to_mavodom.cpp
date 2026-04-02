#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class OdomResampleToMavros
{
public:
    OdomResampleToMavros(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    {
        pnh.param<std::string>("in_odom_topic",  in_odom_topic_,  std::string("/Odometry"));
        pnh.param<std::string>("out_odom_topic", out_odom_topic_, std::string("/mavros/odometry/out"));
        pnh.param<std::string>("world_frame",    world_frame_,    std::string("camera_init"));
        pnh.param<std::string>("body_frame",     body_frame_,     std::string("body"));

        pnh.param<double>("out_rate_hz", out_rate_hz_, 30.0);

        // mode: "hold" or "predict"
        pnh.param<std::string>("mode", mode_, std::string("hold"));

        // 如果 predict：最大外推时间，防止长时间没新数据还在瞎推
        pnh.param<double>("max_predict_dt", max_predict_dt_, 0.15); // 秒

        // 协方差（可调，先给保守默认）
        pnh.param<double>("pos_std_xy", pos_std_xy_, 0.05);
        pnh.param<double>("pos_std_z",  pos_std_z_,  0.10);
        pnh.param<double>("yaw_std",    yaw_std_,    0.10);
        pnh.param<double>("vel_std_xy", vel_std_xy_, 0.10);
        pnh.param<double>("vel_std_z",  vel_std_z_,  0.20);
        pnh.param<double>("wz_std",     wz_std_,     0.20);

        sub_ = nh.subscribe(in_odom_topic_, 50, &OdomResampleToMavros::odomCb, this);
        pub_ = nh.advertise<nav_msgs::Odometry>(out_odom_topic_, 30);

        timer_ = nh.createTimer(ros::Duration(1.0 / out_rate_hz_),
                                &OdomResampleToMavros::timerCb, this);

        ROS_INFO_STREAM("[odom_resample] in_odom_topic  : " << in_odom_topic_);
        ROS_INFO_STREAM("[odom_resample] out_odom_topic : " << out_odom_topic_);
        ROS_INFO_STREAM("[odom_resample] out_rate_hz    : " << out_rate_hz_);
        ROS_INFO_STREAM("[odom_resample] mode           : " << mode_);
        ROS_INFO_STREAM("[odom_resample] frames         : " << world_frame_ << " -> " << body_frame_);
    }

private:
    ros::Subscriber sub_;
    ros::Publisher  pub_;
    ros::Timer      timer_;

    nav_msgs::Odometry last_odom_;
    ros::Time last_odom_stamp_;
    bool have_odom_ = false;

    std::string in_odom_topic_;
    std::string out_odom_topic_;
    std::string world_frame_;
    std::string body_frame_;
    std::string mode_;

    double out_rate_hz_;
    double max_predict_dt_;

    double pos_std_xy_, pos_std_z_, yaw_std_;
    double vel_std_xy_, vel_std_z_, wz_std_;

    static double yawFromQuat(const geometry_msgs::Quaternion& qmsg)
    {
        tf2::Quaternion q(qmsg.x, qmsg.y, qmsg.z, qmsg.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        return yaw;
    }

    static geometry_msgs::Quaternion quatFromYaw(double yaw)
    {
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        geometry_msgs::Quaternion qmsg;
        qmsg.x = q.x(); qmsg.y = q.y(); qmsg.z = q.z(); qmsg.w = q.w();
        return qmsg;
    }

    void fillCov(nav_msgs::Odometry& odom)
    {
        for (int i = 0; i < 36; i++) {
            odom.pose.covariance[i] = 0.0;
            odom.twist.covariance[i] = 0.0;
        }

        odom.pose.covariance[0]  = pos_std_xy_ * pos_std_xy_; // x
        odom.pose.covariance[7]  = pos_std_xy_ * pos_std_xy_; // y
        odom.pose.covariance[14] = pos_std_z_  * pos_std_z_;  // z
        odom.pose.covariance[35] = yaw_std_    * yaw_std_;    // yaw

        odom.twist.covariance[0]  = vel_std_xy_ * vel_std_xy_; // vx
        odom.twist.covariance[7]  = vel_std_xy_ * vel_std_xy_; // vy
        odom.twist.covariance[14] = vel_std_z_  * vel_std_z_;  // vz
        odom.twist.covariance[35] = wz_std_     * wz_std_;     // wz
    }

    void odomCb(const nav_msgs::Odometry::ConstPtr& msg)
    {
        last_odom_ = *msg;
        last_odom_stamp_ = msg->header.stamp;
        have_odom_ = true;
    }

    void timerCb(const ros::TimerEvent& ev)
    {
        if (!have_odom_) return;

        nav_msgs::Odometry out = last_odom_;

        // 统一 frames（MAVROS/EKF 更喜欢明确的 frame）
        out.header.stamp = ros::Time::now();
        out.header.frame_id = world_frame_;
        out.child_frame_id  = body_frame_;

        if (mode_ == "predict") {
            // 用速度做简单外推：x += vx*dt, y += vy*dt, yaw += wz*dt
            double dt = (out.header.stamp - last_odom_stamp_).toSec();
            if (dt < 0) dt = 0; // 防止时间回退
            if (dt > max_predict_dt_) dt = max_predict_dt_; // 防止太久没更新

            const double vx = last_odom_.twist.twist.linear.x;
            const double vy = last_odom_.twist.twist.linear.y;
            const double wz = last_odom_.twist.twist.angular.z;

            double yaw = yawFromQuat(last_odom_.pose.pose.orientation);
            yaw += wz * dt;

            out.pose.pose.position.x = last_odom_.pose.pose.position.x + vx * dt;
            out.pose.pose.position.y = last_odom_.pose.pose.position.y + vy * dt;
            out.pose.pose.position.z = last_odom_.pose.pose.position.z; // 2D车通常不推 z

            out.pose.pose.orientation = quatFromYaw(yaw);
        }

        fillCov(out);
        pub_.publish(out);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_resample_to_mavros");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    OdomResampleToMavros node(nh, pnh);
    ros::spin();
    return 0;
}
