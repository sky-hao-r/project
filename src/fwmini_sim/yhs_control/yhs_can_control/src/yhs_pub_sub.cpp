#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include"yhs_can_msgs/ctrl_cmd.h"

ros::Publisher pub;
// 回调函数，当接收到消息时被调用
void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
        // 这里可以直接发布接收到的消息，也可以进行一些处理后再发布
        yhs_can_msgs::ctrl_cmd cmd_vel;
        cmd_vel.ctrl_cmd_gear = 06;
        cmd_vel.ctrl_cmd_x_linear = msg->linear.x;
        cmd_vel.ctrl_cmd_y_linear= msg->linear.y;
        cmd_vel.ctrl_cmd_z_angular= msg->angular.z/3.14*180;
        pub.publish(cmd_vel);
}

int main(int argc, char *argv[])
{
        ros::init(argc, argv, "fwmini_subscriber");
        ros::NodeHandle nh;
        // 订阅话题cmd_vcmd_vel
        pub = nh.advertise<yhs_can_msgs::ctrl_cmd>("/ctrl_cmd", 10);
        ros::Subscriber sub = nh.subscribe("/four_wheel_steering_controller/cmd_vel", 10, cmdVelCallback);
        ros::spin();

        return 0;
}