#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class OdomToVisionPoseResample
{
public:
    OdomToVisionPoseResample(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    {
        pnh.param<std::string>("odom_topic", odom_topic_, std::string("/Odometry"));
        pnh.param<std::string>("world_frame", world_frame_, std::string("camera_init"));

        pnh.param<double>("out_rate_hz", out_rate_hz_, 30.0);
        pnh.param<std::string>("mode", mode_, std::string("hold")); // "hold" or "predict"
        pnh.param<double>("max_predict_dt", max_predict_dt_, 0.15);

        pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 30);
        odom_sub_ = nh.subscribe(odom_topic_, 50, &OdomToVisionPoseResample::odomCb, this);

        timer_ = nh.createTimer(ros::Duration(1.0 / out_rate_hz_),
                                &OdomToVisionPoseResample::timerCb, this);

        ROS_INFO_STREAM("[odom_to_vision_pose_resample] odom_topic=" << odom_topic_);
        ROS_INFO_STREAM("[odom_to_vision_pose_resample] out_rate_hz=" << out_rate_hz_
                        << " mode=" << mode_ << " max_predict_dt=" << max_predict_dt_);
        ROS_INFO_STREAM("[odom_to_vision_pose_resample] world_frame=" << world_frame_);
    }

private:
    ros::Subscriber odom_sub_;
    ros::Publisher  pose_pub_;
    ros::Timer      timer_;

    std::string odom_topic_;
    std::string world_frame_;
    double out_rate_hz_{30.0};
    std::string mode_{"hold"};
    double max_predict_dt_{0.15};

    nav_msgs::Odometry last_odom_;
    ros::Time last_stamp_;
    bool have_odom_{false};

    // 用于差分速度（给 predict 用）
    nav_msgs::Odometry prev_odom_;
    bool have_prev_{false};
    double vx_{0.0}, vy_{0.0}, wz_{0.0}; // world-frame vx/vy, yaw-rate

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

    static double wrapAngle(double a)
    {
        while (a > M_PI)  a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }

    void odomCb(const nav_msgs::Odometry::ConstPtr& msg)
    {
        if (have_odom_) {
            prev_odom_ = last_odom_;
            have_prev_ = true;
        }

        last_odom_ = *msg;
        last_stamp_ = msg->header.stamp;
        have_odom_ = true;

        // 用相邻两帧 pose 差分估计速度（只给 predict 用）
        if (have_prev_) {
            const double dt = (last_odom_.header.stamp - prev_odom_.header.stamp).toSec();
            if (dt > 1e-3 && dt < 1.0) {
                const double dx = last_odom_.pose.pose.position.x - prev_odom_.pose.pose.position.x;
                const double dy = last_odom_.pose.pose.position.y - prev_odom_.pose.pose.position.y;

                const double yaw_now  = yawFromQuat(last_odom_.pose.pose.orientation);
                const double yaw_prev = yawFromQuat(prev_odom_.pose.pose.orientation);
                const double dyaw = wrapAngle(yaw_now - yaw_prev);

                vx_ = dx / dt;
                vy_ = dy / dt;
                wz_ = dyaw / dt;
            }
        }
    }

    void timerCb(const ros::TimerEvent&)
    {
        if (!have_odom_) return;

        const ros::Time now = ros::Time::now();

        geometry_msgs::Pose pose = last_odom_.pose.pose;

        if (mode_ == "predict") {
            double dt = (now - last_stamp_).toSec();
            if (dt < 0) dt = 0;
            if (dt > max_predict_dt_) dt = max_predict_dt_;

            // 按世界系速度外推位置
            pose.position.x += vx_ * dt;
            pose.position.y += vy_ * dt;

            // 只用 yaw 外推姿态（2D车）
            double yaw = yawFromQuat(pose.orientation);
            yaw = wrapAngle(yaw + wz_ * dt);
            pose.orientation = quatFromYaw(yaw);
        }

        geometry_msgs::PoseStamped out;
        out.header.stamp = now;
        out.header.frame_id = world_frame_.empty() ? last_odom_.header.frame_id : world_frame_;
        out.pose = pose;

        pose_pub_.publish(out);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_to_vision_pose_resample");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    OdomToVisionPoseResample node(nh, pnh);
    ros::spin();
    return 0;
}