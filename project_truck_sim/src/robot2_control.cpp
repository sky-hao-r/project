#include "common_control.h" // 引用公共代码库
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

namespace Robot2_Control
{
        class Robot2Control : public Common_Control::CommonControl
        {
        public:
                Robot2Control(const std::string &robot_name, const std::string &odom_topic, const std::string &robot_topic, const std::string &pose_pub_topic)
                    : CommonControl(robot_name, odom_topic, robot_topic, pose_pub_topic)
                {

                }
                void Init()
                {
                        ros::NodeHandle nh;
                        robot1_pose_sub = nh.subscribe("robot1_pose", 1, &Robot2Control::Robot1PoseCallback, this);
                        marker1_pub = nh.advertise<visualization_msgs::Marker>("visualization1_marker", 1);
                }

                void Run()
                {
                        ros::Rate rate(100); // 设置循环频率
                        while (ros::ok())
                        {
                                ros::spinOnce(); // 处理ROS回调
                                if(robot1_path_vector.empty())
                                {
                                        ROS_INFO("robot1_path_vector is empty,waiting");
                                        rate.sleep();
                                        continue;
                                }
                                switch (Robot_State)
                                {
                                        case Common_Control::StateValue::Idle:
                                                // 开始运行前的准备
                                                Robot_State = Common_Control::StateValue::Run;
                                                current_index = 0; // 初始化当前路径点索引
                                                ROS_INFO("Robot2 is transitioning to Run state...");
                                                // 立即进入下一个状态
                                                break;

                                        case Common_Control::StateValue::Run:
                                                Running_State();
                                                break;

                                        default:
                                                ROS_ERROR("Unknown Robot State!");
                                                break;
                                }

                                rate.sleep();
                        }
                }

        private:
                ros::Subscriber robot1_pose_sub; // 订阅机器人1的路径点话题
                Path_Type robot1_path_vector;    // 存储机器人1的路径点
                ros::Publisher marker1_pub;

                void Running_State()
                {
                        // 获取当前目标路径点
                        path_type current_goal = robot1_path_vector[current_index];

                        // 检查运行状态
                        switch (Run_State)
                        {
                        case Common_Control::RunStateValue::Goto:
                                // 前往第一个路径点
                                Goto_Goal({current_goal});
                                break;

                        case Common_Control::RunStateValue::Tracking:
                                // 轨迹跟踪
                                if (current_index < robot1_path_vector.size())
                                {
                                        Tracking_Path(robot1_path_vector);
                                }
                                else
                                {
                                        ROS_INFO("Robot2 has completed tracking the path.");
                                        Robot_State = Common_Control::StateValue::Idle;
                                        Run_State = Common_Control::RunStateValue::Goto;
                                }
                                break;
                        case Common_Control::RunStateValue::Follow:
                                Stop(); // 停止机器人
                                break;

                        default:
                                ROS_ERROR("Unknown Run State!");
                                break;
                        }
                }

                void Goto_Goal(Path_Type path_point)
                {
                        if (path_point.empty())
                                return;

                        double target_x = path_point[0].first.first;
                        double target_y = path_point[0].first.second;
                        double target_yaw = path_point[0].second;

                        ros::spinOnce();
                        ROS_INFO("Goto_Goal: %f, %f, %f", target_x, target_y, target_yaw);
                        ROS_INFO("Robot_Pose: %f, %f, %f", robot_pose.x, robot_pose.y, robot_pose.yaw);

                        double distance = sqrt(pow(target_x - robot_pose.x, 2) + pow(target_y - robot_pose.y, 2));
                        ROS_INFO("distance: %f", distance);
                        if (distance >= 0.05)
                        {

                                double angle_to_goal = atan2(target_y - robot_pose.y, target_x - robot_pose.x);

                                geometry_msgs::Twist vel_msg;
                                vel_msg.angular.z = 1 * (angle_to_goal - robot_pose.yaw);
                                vel_msg.linear.x = 0.5;

                                vel_pub.publish(vel_msg);
                        }
                        else
                        {
                                ROS_INFO("yaw_err: %f", std::abs(target_yaw - robot_pose.yaw));
                                if (std::abs(target_yaw - robot_pose.yaw) > 0.05)
                                {

                                        geometry_msgs::Twist vel_msg;
                                        vel_msg.angular.z = 0.5;
                                        vel_msg.linear.x = 0.0;
                                        vel_pub.publish(vel_msg);
                                }
                                else
                                {

                                        geometry_msgs::Twist vel_msg;
                                        vel_msg.angular.z = 0;
                                        vel_msg.linear.x = 0.0;
                                        vel_pub.publish(vel_msg);

                                        ROS_INFO("goto goal success");
                                        goto_complete_flag = true;
                                        Run_State = Common_Control::RunStateValue::Tracking;
                                }
                        }
                        // rate.sleep();
                }

                void Tracking_Path(Path_Type path)
                {
                        // 获取当前位置与目标路径点
                        path_type current_point = {{robot_pose.x, robot_pose.y}, robot_pose.yaw};
                        int nearest_index = GetMinDistance(current_point, path); // 找到最近的路径点索引

                        // 更新当前路径点索引
                        if (nearest_index > current_index)
                        {
                                current_index = nearest_index; // 更新为最近的路径点索引
                        }

                        // 目标点
                        path_type goal_point = path[current_index];

                        // 判断是否到达最后一个路径点
                        if (current_index == path.size() - 1)
                        {
                                ROS_INFO("Reached the last point. Stopping the robot.");
                                goto_complete_flag = false;
                                Run_State = Common_Control::RunStateValue::Follow;
                                return;
                        }

                        // 计算横向误差 e_y
                        double e_y = Get_Distance(current_point, goal_point);
                        e_y = ((current_point.first.second - goal_point.first.second) * cos(goal_point.second) -
                               (current_point.first.first - goal_point.first.first) * sin(goal_point.second)) <= 0
                                  ? e_y
                                  : -e_y;

                        // 计算航向误差 theta_e
                        double theta_e = goal_point.second - current_point.second;

                        // 控制参数
                        double kp = 1.0;          // 比例增益
                        double truck_speed = 0.1; // 线速度

                        // 计算转向角 delta
                        double delta_e = atan2(kp * e_y, truck_speed);
                        double delta = delta_e + theta_e;

                        // 归一化 delta 到 [-π, π]
                        while (delta > M_PI)
                                delta -= 2 * M_PI;
                        while (delta < -M_PI)
                                delta += 2 * M_PI;

                        // 发布速度指令
                        geometry_msgs::Twist vel_msg;
                        vel_msg.linear.x = truck_speed;
                        vel_msg.angular.z = delta;

                        if (odom_callback_flag) // 确保有里程计数据
                        {
                                vel_pub.publish(vel_msg);
                        }

                        ROS_INFO("delta: %f", delta);
                }

                void Robot1PoseCallback(const project_truck::RobotPose::ConstPtr &msg)
                {
                        // 将接收到的路径点添加到路径向量中
                        robot1_path_vector.push_back({{msg->x, msg->y}, msg->yaw});
                        // 可视化机器人1的路径点
                        Visual_Path(robot1_path_vector);
                }

                void Visual_Path(const Path_Type &path)
                {
                        visualization_msgs::Marker marker;
                        marker.header.frame_id = "odom";
                        marker.header.stamp = ros::Time::now();
                        marker.ns = robot_name;
                        marker.id = 0;
                        marker.type = visualization_msgs::Marker::LINE_STRIP;
                        marker.action = visualization_msgs::Marker::ADD;
                        marker.lifetime = ros::Duration();
                        marker.color.r = 0.0;
                        marker.color.b = 1.0;
                        marker.color.g = 1.0;
                        marker.color.a = 1.0;
                        marker.scale.x = 0.02;
                        marker.pose.orientation.w = 1.0;

                        for (const auto &point : path)
                        {
                                geometry_msgs::Point p;
                                p.x = point.first.first;
                                p.y = point.first.second;
                                p.z = 0;
                                marker.points.push_back(p);
                        }
                        marker1_pub.publish(marker);
                }
        };
}

int main(int argc, char *argv[])
{
        ros::init(argc, argv, "second_car_control");

        std::string robot_name = "second_car";
        std::string odom_topic = "/four_wheel_steering_controller/odom";
        std::string robot_topic = "/four_wheel_steering_controller/cmd_vel";
        std::string pose_pub_topic = "robot2_pose";

        Robot2_Control::Robot2Control robot2(robot_name, odom_topic, robot_topic, pose_pub_topic);
        robot2.Init(); // 初始化
        robot2.Run();  // 运行

        return 0;
}