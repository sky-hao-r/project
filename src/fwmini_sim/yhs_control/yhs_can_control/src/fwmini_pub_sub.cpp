#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

ros::Publisher pub;
// 回调函数，当接收到消息时被调用
void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
        // 这里可以直接发布接收到的消息，也可以进行一些处理后再发布
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = msg->linear.x;
        cmd_vel.linear.y = msg->linear.y;
        cmd_vel.angular.z = msg->angular.z;
        pub.publish(cmd_vel);
}

int main(int argc, char *argv[])
{
        ros::init(argc, argv, "fwmini_subscriber");
        ros::NodeHandle nh;
        // 订阅话题cmd_vcmd_vel
        pub = nh.advertise<geometry_msgs::Twist>("/four_wheel_steering_controller/cmd_vel", 10);
        ros::Subscriber sub = nh.subscribe("/vel", 10, cmdVelCallback);
        ros::spin();

        return 0;
}