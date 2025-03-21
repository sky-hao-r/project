#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/String.h"
#include <tf/tf.h>
#include <cmath>
#include <vector>
#include <string>
#include "project_truck/hmh_task.h" // 包含任务服务的头文件
#include "project_truck/RobotPose.h"
#include "odom1/underpan_speed.h"
#include "odom1/underpan_code_w.h"

using path_type = std::pair<std::pair<double, double>, double>;
using Path_Type = std::vector<path_type>;

namespace Common_Control
{
        /*枚举类型*/
        enum class StateValue : uint8_t
        {
                Idle = 0, // 空闲状态
                Record,   // 记录状态
                Run,      // 运行状态
                Pause     // 暂停状态
        };

        enum class RunStateValue : uint8_t
        {
                Goto = 0,   // 前往状态
                Tracking,   // 跟踪状态
                Waitting,     // 等待状态
                Stop // 跟随状态
        };

        class CommonControl
        {
        public:
                CommonControl(const std::string &robot_name, const std::string &odom_topic, const std::string &robot_topic, const std::string &pose_pub_topic);
                ~CommonControl();

                void Init();
                Path_Type ReadFile(const std::string &filePath); // 读取路径文件
                void Visual_Path(const Path_Type &path);         // 可视化路径
                void Goto_Goal(const path_type &goal_point, double tolerance , double speed, double angular_gain,bool is_goto);
                void Tracking_Path_pure_pursuit(const Path_Type &path_vector, double ld, double speed, double angular_gain, bool is_robot1);
                bool task_srv(project_truck::hmh_task::Request &req, project_truck::hmh_task::Response &resp);

        protected:
                std::string robot_name;
                std::string path_topic;
                std::string odom_topic;
                std::string robot_topic;
                std::string pose_pub_topic;

                project_truck::RobotPose robot_pose_msg;

                Path_Type path_vector;
                path_type current_point;
                path_type goal_point;
                StateValue Robot_State;  // 机器人状态
                RunStateValue Run_State; // 运行状态

                int current_index = 0;
                bool odom_callback_flag; // 回调标志位
                bool goto_complete_flag; // 到达标志位
                bool robot_type;

                double fl = 0.0, fr = 0.0, B = 2.9; // fl和fr分别为左右轮滑移参数,B为轮距

                ros::ServiceServer task_srv_; // 服务服务器

                ros::Subscriber odom_sub;
                ros::Subscriber path_sub;

                ros::Publisher vel_pub;
                ros::Publisher marker_pub;
                ros::Publisher pose_pub; // 发布路径点

                struct RobotPose
                {
                        double x, y, yaw;
                };
                RobotPose robot_pose;

                void OdomCallback_true(const geometry_msgs::PoseStamped::ConstPtr &msg);
                double Stand_base_x(const double x);
                double Stand_base_y(const double y);
                double Get_Distance(path_type point1, path_type point2);            // 计算两点之间的距离
                int GetMinDistance(path_type current_point, Path_Type path_vector); // 获取当前点到路径上所有点的最小距离
                void Stop();
        };
}