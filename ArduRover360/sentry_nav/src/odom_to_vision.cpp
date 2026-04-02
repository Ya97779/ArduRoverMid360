#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

class OdomToVision
{

private:
    ros::Subscriber odom_sub_;
    ros::Publisher pose_pub_;
    ros::Publisher speed_pub_;
    std::string odom_topic_;
    bool publish_speed_;

    

public:
    OdomToVision(ros::NodeHandle &nh, ros::NodeHandle &pnh)
    {
        // 从参数服务器读取 odom 话题名，默认为 /Odometry
        pnh.param<std::string>("odom_topic", odom_topic_, std::string("/Odometry"));
        pnh.param<bool>("publish_speed", publish_speed_, true);

        pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
        if (publish_speed_)
        {
            speed_pub_ = nh.advertise<geometry_msgs::TwistStamped>("/mavros/vision_speed/speed_twist", 10);
        }

        odom_sub_ = nh.subscribe(odom_topic_, 50, &OdomToVision::odomCallback, this);

        ROS_INFO_STREAM("[odom_to_vision] Subscribing odom from: " << odom_topic_);
        ROS_INFO_STREAM("[odom_to_vision] Publishing pose to: /mavros/vision_pose/pose");
        if (publish_speed_)
        {
            ROS_INFO_STREAM("[odom_to_vision] Publishing speed to: /mavros/vision_speed/speed_twist");
        }
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        // 1) Pose → vision_pose
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header = msg->header;
        // 建议确保 frame_id 是统一的世界系（比如 "camera_init" ）
        pose_msg.pose = msg->pose.pose;
        pose_pub_.publish(pose_msg);

        // 2) Twist → vision_speed（可选）
        if (publish_speed_)
        {
            geometry_msgs::TwistStamped twist_msg;
            twist_msg.header = msg->header;
            twist_msg.twist = msg->twist.twist;
            speed_pub_.publish(twist_msg);
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_to_vision");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    OdomToVision node(nh, pnh);

    ros::spin();
    return 0;
}
