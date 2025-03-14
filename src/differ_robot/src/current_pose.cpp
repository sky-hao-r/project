#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include"tf2_geometry_msgs/tf2_geometry_msgs.h"
#include"tf2/utils.h"

ros::Publisher pub_pose1;
ros::Publisher pub_pose2;
ros::Publisher pub_pose3;
ros::Publisher pub_pose4;
ros::Publisher pub_pose5;

void odomCallback1(const nav_msgs::Odometry::ConstPtr &msg)
{
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header = msg->header;
        pose_msg.pose.position = msg->pose.pose.position;

        pose_msg.pose.orientation.z=tf2::getYaw(msg->pose.pose.orientation);

        pub_pose1.publish(pose_msg);
}
void odomCallback2(const nav_msgs::Odometry::ConstPtr &msg)
{
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header = msg->header;
        pose_msg.pose.position = msg->pose.pose.position;

        pose_msg.pose.orientation.z = tf2::getYaw(msg->pose.pose.orientation);

        pub_pose2.publish(pose_msg);
}
void odomCallback3(const nav_msgs::Odometry::ConstPtr &msg)
{
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header = msg->header;
        pose_msg.pose.position = msg->pose.pose.position;

        pose_msg.pose.orientation.z = tf2::getYaw(msg->pose.pose.orientation);

        pub_pose3.publish(pose_msg);
}
void odomCallback4(const nav_msgs::Odometry::ConstPtr &msg)
{
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header = msg->header;
        pose_msg.pose.position = msg->pose.pose.position;

        pose_msg.pose.orientation.z = tf2::getYaw(msg->pose.pose.orientation);

        pub_pose4.publish(pose_msg);
}
void odomCallback5(const nav_msgs::Odometry::ConstPtr &msg)
{
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header = msg->header;
        pose_msg.pose.position = msg->pose.pose.position;

        pose_msg.pose.orientation.z = tf2::getYaw(msg->pose.pose.orientation);

        pub_pose5.publish(pose_msg);
}

int main(int argc, char **argv)
{
        ros::init(argc, argv, "curren_pose");
        ros::NodeHandle nh;

        pub_pose1 = nh.advertise<geometry_msgs::PoseStamped>("/robot1/current_pose", 10);
        pub_pose2 = nh.advertise<geometry_msgs::PoseStamped>("/robot2/current_pose", 10);
        pub_pose3 = nh.advertise<geometry_msgs::PoseStamped>("/robot3/current_pose", 10);
        pub_pose4 = nh.advertise<geometry_msgs::PoseStamped>("/robot4/current_pose", 10);
        pub_pose5 = nh.advertise<geometry_msgs::PoseStamped>("/robot5/current_pose", 10);

        ros::Subscriber sub_odom1 = nh.subscribe("/robot1/base_pose_truth", 10, odomCallback1);
        ros::Subscriber sub_odom2 = nh.subscribe("/robot2/base_pose_truth", 10, odomCallback2);
        ros::Subscriber sub_odom3 = nh.subscribe("/robot3/base_pose_truth", 10, odomCallback3);
        ros::Subscriber sub_odom4 = nh.subscribe("/robot4/base_pose_truth", 10, odomCallback4);
        ros::Subscriber sub_odom5 = nh.subscribe("/robot5/base_pose_truth", 10, odomCallback5);

        ros::spin();

        return 0;
}