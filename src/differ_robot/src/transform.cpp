#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h> // 用于四元数和欧拉角的转换
#include "differ_robot/PoseRPY.h"

class OdomToPoseRPY
{
private:
        ros::NodeHandle nh;       // ROS 节点句柄
        ros::Subscriber odom_sub; // 订阅者
        ros::Publisher pub;       // 发布者

public:
        OdomToPoseRPY()
        {
                // 初始化订阅者和发布者
                odom_sub = nh.subscribe("odom", 10, &OdomToPoseRPY::odomCallback, this);
                pub = nh.advertise<differ_robot::PoseRPY>("pose_rpy", 10);
        }

        // 回调函数：处理接收到的odom消息
        void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
        {
                // 创建自定义消息
                differ_robot::PoseRPY pose_rpy;

                // 提取位置信息
                pose_rpy.x = msg->pose.pose.position.x;
                pose_rpy.y = msg->pose.pose.position.y;
                pose_rpy.z = msg->pose.pose.position.z;

                // 提取四元数并转换为RPY角
                tf::Quaternion q(
                    msg->pose.pose.orientation.x,
                    msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z,
                    msg->pose.pose.orientation.w);
                tf::Matrix3x3 m(q);
                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);

                pose_rpy.roll = roll;
                pose_rpy.pitch = pitch;
                pose_rpy.yaw = yaw;

                // 发布自定义消息
                pub.publish(pose_rpy);
        }
};

int main(int argc, char **argv)
{
        // 初始化节点
        ros::init(argc, argv, "odom_to_pose_rpy");

        // 创建类实例
        OdomToPoseRPY odom_to_pose_rpy;

        // 运行节点
        ros::spin();

        return 0;
}