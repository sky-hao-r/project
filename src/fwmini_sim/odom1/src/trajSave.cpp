#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <fstream>
#include <sstream>
// #include <nlohmann/json.hpp>  // 用于 JSON 格式的库

class TrajectorySaver
{
public:
    TrajectorySaver()
    {
        pose_sub_ = nh_.subscribe("/current_pose", 10, &TrajectorySaver::poseInsCallback, this);
        // lidar_pose_sub_ = nh_.subscribe("/Odometry", 10, &TrajectorySaver::poseLidarCallback, this);
        trajectory_ins_.clear();
        // trajectory_lidar_.clear();
        save_timer_ = nh_.createTimer(ros::Duration(10.0), &TrajectorySaver::saveTrajectory, this);
        csv_file_.open("trajectory_ins.csv", std::ios::app);
        if (!csv_file_.is_open())
        {
            ROS_ERROR("Failed to open CSV file for saving trajectory_ins.");
        }
        // csv_file_lidar_.open("trajectory_lidar.csv", std::ios::app);
        // if (!csv_file_lidar_.is_open())
        // {
        //     ROS_ERROR("Failed to open CSV file for saving trajectory_lidar.");
        // }
    }

    ~TrajectorySaver()
    {
        // 确保程序结束时关闭文件
        if (csv_file_.is_open())
            csv_file_.close();
        // if (csv_file_lidar_.is_open())
        //     csv_file_lidar_.close();
    }

    // 订阅回调函数
    void poseInsCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        trajectory_ins_.push_back(*msg);  // 保存接收到的每个PoseStamped消息
    }
    // void poseLidarCallback(const nav_msgs::Odometry::ConstPtr& msg)
    // {
    //     trajectory_lidar_.push_back(*msg);  // 保存接收到的每个PoseStamped消息
    // }

    // 保存轨迹数据到文件
    void saveTrajectory(const ros::TimerEvent&)
    {
        if (!trajectory_ins_.empty())
        {
            // 写入 CSV 文件
            saveToCSV();
            // 清空轨迹容器
            trajectory_ins_.clear();
            // trajectory_lidar_.clear();
        }
        else
        {
            ROS_WARN("No trajectory data to save.");
        }
    }

private:
    // 保存轨迹到 CSV 文件
    void saveToCSV()
    {
        for (const auto& pose : trajectory_ins_)
        {
            if (csv_file_.is_open())
            {
                csv_file_ << std::fixed << std::setprecision(6) // 保留 8 位小数
                          << pose.header.stamp.toSec() << " "
                          << pose.pose.position.x << " "
                          << pose.pose.position.y << " "
                          << pose.pose.position.z << " "
                          //   << pose.pose.orientation.x << " "
                          //   << pose.pose.orientation.y << " "
                          << pose.pose.orientation.z << "\n";
                //   << pose.pose.orientation.w << "\n";
            }
            else
            {
                ROS_ERROR("CSV file is not open for saving data.");
            }
        }
        // for (const auto& pose : trajectory_lidar_)
        // {
        //     if (csv_file_lidar_.is_open())
        //     {
        //         csv_file_lidar_ << std::fixed << std::setprecision(8) // 保留 8 位小数
        //                         << pose.header.stamp.toSec() << ","
        //                         << pose.pose.pose.position.x << ","
        //                         << pose.pose.pose.position.y << ","
        //                         << pose.pose.pose.position.z << ","
        //                         // << pose.pose.pose.orientation.x << ","
        //                         // << pose.pose.pose.orientation.y << ","
        //                         << pose.pose.pose.orientation.z << "\n";
        //         //   << pose.pose.pose.orientation.w << "\n";
        //     }
        //     else
        //     {
        //         ROS_ERROR("CSV file is not open for saving data.");
        //     }
        // }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber pose_sub_;
    // ros::Subscriber lidar_pose_sub_;
    ros::Timer save_timer_;
    std::vector<geometry_msgs::PoseStamped> trajectory_ins_;
    // std::vector<nav_msgs::Odometry> trajectory_lidar_;
    std::ofstream csv_file_;
    // std::ofstream csv_file_lidar_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "trajectory_saver");
    TrajectorySaver saver;

    ros::spin();
    
    return 0;
}