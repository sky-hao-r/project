#pragma once

#include <cmath>
#include <iostream>
#include <vector>
#include <random>
#include <sstream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include <fstream>
#include "geometry_msgs/PoseStamped.h"
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include "project_truck/robot_moveAction.h"
#include <thread>
#include"yhs_can_msgs/ctrl_cmd.h"
#include <geometry_msgs/Twist.h>
#include "project_truck/hmh_task.h"

using path_type = std::pair<std::pair<double, double>, double>;
using Path_Type = std::vector<path_type>;

namespace State_Control
{
        /*
        NODE_NAME：ROS节点的名称，为 "hmh_state_control"。
        RVIZ_POINT：点击点话题的名称，为 "/clicked_point"。
        TASK_SRV：任务服务的名称。
        DEG2RAD：度到弧度的转换常量。
        WAIT_COUNT：等待计数，用于等待某些操作完成。
*/
        constexpr static const char *NODE_NAME = "hmh_state_control";
        constexpr static const char *RVIZ_POINT = "/clicked_point";
        constexpr static const char *TASK_SRV = "hmh_task";
        constexpr static const char *PATH_NAME = "/home/hmh/wmm_ws/src/connect_robot/project_truck/Data/input_1.csv";
        // constexpr static const char *PATH_NAME = "/home/hmh/wmm_ws/src/connect_robot/project_truck/Data/trajectory_test.csv";

        constexpr static const char *VELPUB_SIM = "/four_wheel_steering_controller/cmd_vel";
        constexpr static const char *VELPUB_TRUE = "/ctrl_cmd";
        constexpr static const char *Robot_1_vel_topic = "/wmm_1/four_wheel_steering_controller/cmd_vel";
        constexpr static const char *Robot_2_vel_topic = "/wmm_2/four_wheel_steering_controller/cmd_vel";


        constexpr static const char *Odom_SIM = "/four_wheel_steering_controller/odom";
        constexpr static const char *Odom_TRUE = "/current_pose";
        constexpr static const char *Robot_1_odom_topic = "/wmm_1/four_wheel_steering_controller/odom";
        constexpr static const char *Robot_2_odom_topic = "/wmm_2/four_wheel_steering_controller/odom";

        constexpr static const double DEG2RAD = M_PI / 180;
        constexpr static const int WAIT_COUNT = 5 * 10;                            // 5s


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
                Goto = 0, // 前往状态
                Tracking, // 跟踪状态
                Follow,   // 跟随状态
                Wait      // 等待状态
        };

        class StateControl
        {
                public:
                        double kp = 1;
                        double truck_speed = 0.1;
                        double goal_speed=0.3;
                        double kd_g = 0.4;

                public:
                        StateControl();  // 构造函数，用于初始化状态机
                        ~StateControl(); // 析构函数，用于释放资源
                        void Init();     // 初始化状态机，设置ROS节点、订阅者、发布者、服务等
                        void Run();      // 运行状态机，根据当前状态执行相应的行为。
                        void Stop();     // 停止状态机
                private:
                        Path_Type ReadFile(const std::string &filePath);
                        void Visual_Path(Path_Type path);
                        void Pub_Robot_Path(const nav_msgs::Odometry::ConstPtr &odom);
                        void Running_State();
                        void Goto_Goal(Path_Type path_point);
                        void Tracking_Path();
                        Path_Type GetFirstPoint(Path_Type path);
                        void OdomCallback_sim(const nav_msgs::Odometry::ConstPtr &msg);
                        void OdomCallback_true(const geometry_msgs::PoseStamped::ConstPtr &msg);
                        bool task_srv(project_truck::hmh_task::Request &req,
                                project_truck::hmh_task::Response &res); // 任务服务回调函数
                        double Get_Distance(path_type point1, path_type point2);
                        int GetMinDistance(path_type current_point, Path_Type path_vector);
                        void reset();

                private:
                        bool robot_type; // 机器人类型，true为真实机器人，false为仿真机器人
                        ros::Publisher vel_pub_,
                        robot_1_vel_pub_,
                        robot_2_vel_pub_,
                        marker_pub_,
                        path_pub_,
                        robot_pos_pub_,
                        robot_path_pub_;
                        ros::Subscriber odom_sub_,
                            robot_1_odom_sub_,
                            robot_2_odom_sub_;
                        ros::ServiceServer task_srv_; // 服务服务器
                        std::string file_path_;// 文件路径
                        StateValue Robot_State; // 机器人状态
                        RunStateValue Run_State; // 运行状态
                        Path_Type path_vector; // 路径向量
                        path_type current_point; // 当前点
                        path_type goal_point; // 目标点
                        int current_index; // 索引的最近点
                        bool odom_callback_flag; // 回调标志位
                        nav_msgs::Path path;//小车的实时路径

                        struct RobotPose
                        {
                                float x, y, yaw;
                        }; // 机器人位置信息
                        RobotPose robot_pose; // 机器人位置信息
                        RobotPose robot_1_pose; // 机器人1位置信息
                        RobotPose robot_2_pose; // 机器人2位置信息
        };
}
