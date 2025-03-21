#include "project_truck/common_control.h"
#include <chrono>
#include <fstream>

namespace Common_Control
{
        CommonControl::CommonControl(const std::string &robot_name, const std::string &odom_topic, const std::string &robot_topic, const std::string &pose_pub_topic)
            : robot_name(robot_name), odom_topic(odom_topic), robot_topic(robot_topic), pose_pub_topic(pose_pub_topic),
              odom_callback_flag(false),
              goto_complete_flag(false),
              Robot_State(StateValue::Idle),
              Run_State(RunStateValue::Goto) // 车的运行状态
        {

        }

        CommonControl::~CommonControl()
        {
        }

        void CommonControl::Init()
        {
                ros::NodeHandle nh;
                odom_sub = nh.subscribe(odom_topic, 1, &CommonControl::OdomCallback_true, this);
                vel_pub = nh.advertise<odom1::underpan_speed>(robot_topic, 1);
                marker_pub = nh.advertise<visualization_msgs::Marker>("path_marker", 1);
                pose_pub = nh.advertise<project_truck::RobotPose>(pose_pub_topic, 1);
                task_srv_ = nh.advertiseService("hmh_task", &CommonControl::task_srv, this);
        }

        Path_Type CommonControl::ReadFile(const std::string &filePath)
        {
                Path_Type path_vector;
                std::ifstream file(filePath);
                if (!file.is_open())
                {
                        std::cerr << "无法打开文件: " << filePath << std::endl;
                        return path_vector;
                }

                std::string line;
                while (std::getline(file, line))
                {
                        std::istringstream ss(line);
                        std::string token;
                        std::vector<std::string> tokens;
                        // Split the line by commas
                        while (std::getline(ss, token, ','))
                        {
                                tokens.push_back(token);
                        }
                        // Check if the line has the correct number of tokens
                        if (tokens.size() != 4)
                        {
                                std::cerr << "Skipping invalid line: " << line << std::endl;
                                continue;
                        }

                        double x = std::stod(tokens[0])+0.78593;
                        double y = std::stod(tokens[1])-2.65154;
                        double z = std::stod(tokens[2]);
                        geometry_msgs::Point path_point;
                        path_point.x = x;
                        path_point.z = 0;

                        double yaw = std::stod(tokens[3]) / 180.0 * M_PI;

                        if(yaw>M_PI)
                        {
                                yaw-=2*M_PI;
                        }
                        else if(yaw<-M_PI)
                        {
                                yaw+=2*M_PI;
                        }

                        path_vector.push_back({{x, y}, yaw});
                }
                return path_vector;
        }

        void CommonControl::Visual_Path(const Path_Type &path)
        {
                visualization_msgs::Marker marker;
                marker.header.frame_id = "world";
                marker.header.stamp = ros::Time::now();
                marker.ns = robot_name;
                marker.id = 0;
                marker.type = visualization_msgs::Marker::LINE_STRIP;
                marker.action = visualization_msgs::Marker::ADD;
                marker.lifetime = ros::Duration();
                marker.color.r = 1.0;
                marker.color.b = 0.0;
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
                marker_pub.publish(marker);
        }

        void CommonControl::Goto_Goal(const path_type &goal_point, double tolerance, double speed, double angular_gain,bool is_goto)
        {
                double target_x = goal_point.first.first;
                double target_y = goal_point.first.second;
                double target_yaw = goal_point.second;

                ROS_INFO("Goto_Goal: %f, %f, %f", target_x, target_y, target_yaw);
                ROS_INFO("Robot_Pose: %f, %f, %f", robot_pose.x, robot_pose.y, robot_pose.yaw);

                double distance = sqrt(pow(target_x - robot_pose.x, 2) + pow(target_y - robot_pose.y, 2));
                ROS_INFO("distance: %f", distance);
                if (distance >= tolerance)
                {
                        double angle_to_goal = CommonControl::Stand_base_y(atan2(target_y - robot_pose.y, target_x - robot_pose.x));

                        ROS_INFO("angle_to_go:=%f", angle_to_goal);
                        ROS_INFO("w:=%f", angle_to_goal - robot_pose.yaw);

                        geometry_msgs::Twist vel;
                        vel.linear.x = speed;
                        vel.angular.z = angular_gain * (angle_to_goal - robot_pose.yaw);
                        ROS_INFO("v: %f, w: %f", vel.linear.x, vel.angular.z);
                        odom1::underpan_speed vel_msg;
                        vel_msg.left_speed = (2 * vel.linear.x - B * vel.angular.z) * (1 - fl) / 2; // 左轮速度
                        vel_msg.right_speed = (2 * vel.linear.x + B * vel.angular.z) / ((1 - fr) * 2); // 右轮速度

                        ROS_INFO("left_speed: %f, right_speed: %f", vel_msg.left_speed, vel_msg.right_speed);
                        vel_pub.publish(vel_msg);
                }
                else
                {
                        ROS_INFO("yaw_err: %f", std::abs(target_yaw - robot_pose.yaw));
                        // ros::Duration(1).sleep();
                        if (is_goto)
                        {
                                Run_State = Common_Control::RunStateValue::Tracking;
                        }
                        else
                        {
                                Run_State = Common_Control::RunStateValue::Stop;
                        }
                        
                        ROS_INFO("Reached goal point.");
                }
        }

        void CommonControl::Tracking_Path_pure_pursuit(const Path_Type &path_vector, double ld, double speed, double angular_gain, bool is_robot1)
        {
                // 更新当前点的位置和姿态
                current_point = {{robot_pose.x, robot_pose.y}, robot_pose.yaw};

                // 找到离当前点最近的路径点索引
                current_index = GetMinDistance(current_point, path_vector);

                // 检查是否到达终点
                if (current_index == path_vector.size() - 1)
                {
                        ROS_INFO("Reached the last point. Stopping the robot.");
                        goto_complete_flag = false;
                        if (is_robot1)
                        {
                                Run_State = Common_Control::RunStateValue::Waitting;
                        }
                        else
                        {
                                Run_State = Common_Control::RunStateValue::Stop;
                        }
                        return;
                }

                // 从 current_index 开始搜索满足条件的路径点
                int start_index = current_index;
                path_type ahead_path_point;
                while (true)
                {
                        // 获取当前路径点
                        ahead_path_point = path_vector[current_index];

                        // 计算超前点与当前路径点的距离
                        double dis = Get_Distance(current_point, ahead_path_point);
                        if (dis >= ld)
                        {
                                // 找到满足条件的前视距离点
                                break;
                        }

                        // 向后查找下一个路径点
                        current_index += 1;
                        if (current_index >= path_vector.size())
                        {
                                // 循环路径起点
                                current_index = 0;
                                // 如果循环了一整圈仍未找到，重置到初始索引并退出
                                if (current_index == start_index)
                                {
                                        ROS_WARN("Unable to find a path point with distance >= ld. Using current closest point.");
                                        current_index = start_index;
                                        ahead_path_point = path_vector[current_index];
                                        break;
                                }
                        }
                }

                ROS_INFO_STREAM("current_index:" << current_index);
                ROS_INFO("goal_point:x:=%f,y:=%f,yaw:=%f", ahead_path_point.first.first, ahead_path_point.first.second, ahead_path_point.second);
                ROS_INFO("current_point:x:=%f,y:=%f,yaw:=%f", current_point.first.first, current_point.first.second, current_point.second);
                
                // 计算转向角度
                double come_goal_yaw = CommonControl::Stand_base_y(std::atan2(ahead_path_point.first.second - current_point.first.second,ahead_path_point.first.first - current_point.first.first));
                double alpha = come_goal_yaw - current_point.second;
                double delta = std::atan2(2 * speed * sin(alpha), ld);
                ROS_INFO("come_goal_yaw:=%f", come_goal_yaw);
                ROS_INFO("alpha:=%f", alpha);
                ROS_INFO("w:=%f", delta);

                if (delta > M_PI)
                {
                        delta -= 2 * M_PI;
                }
                else if (delta < -M_PI)
                {
                        delta += 2 * M_PI;
                }

                geometry_msgs::Twist vel;
                vel.linear.x = speed;
                vel.angular.z = angular_gain * delta;
                odom1::underpan_speed vel_msg;
                vel_msg.left_speed = (2 * vel.linear.x - B * vel.angular.z) * (1 - fl) / 2;    // 左轮速度
                vel_msg.right_speed = (2 * vel.linear.x + B * vel.angular.z) / ((1 - fr) * 2); // 右轮速度
                if (odom_callback_flag)
                {
                        vel_pub.publish(vel_msg);
                }
        }

        void CommonControl::OdomCallback_true(const geometry_msgs::PoseStamped::ConstPtr &msg)
        {
                odom_callback_flag = true;
                robot_pose.x = msg->pose.position.x;
                robot_pose.y = msg->pose.position.y;
                robot_pose.yaw = msg->pose.orientation.z;
                // robot_pose.yaw = msg->pose.orientation.z;
                robot_pose.yaw = CommonControl::Stand_base_y(msg->pose.orientation.z);

                robot_pose_msg.x = robot_pose.x;
                robot_pose_msg.y = robot_pose.y;
                robot_pose_msg.yaw = robot_pose.yaw;
                pose_pub.publish(robot_pose_msg);
        }

        double CommonControl::Get_Distance(path_type point1, path_type point2)
        {
                return sqrt(pow(point1.first.first - point2.first.first, 2) + pow(point1.first.second - point2.first.second, 2));
        }

        int CommonControl::GetMinDistance(path_type current_point, Path_Type path_vector)
        {
                double min_dis = std::numeric_limits<double>::max();
                int index = 0;
                for (size_t i = 0; i < path_vector.size(); ++i)
                {
                        double distance = Get_Distance(path_vector[i], current_point);
                        if (distance < min_dis)
                        {
                                min_dis = distance;
                                index = i;
                        }
                }
                return index;
        }
        double CommonControl::Stand_base_x(double x)
        {
                double x1 = x+M_PI/2;
                if (x1 > M_PI)
                {
                        x1 -= 2 * M_PI;
                }
                else if (x1 < -M_PI)
                {
                        x1 += 2 * M_PI;
                }
                return x1;
        }
        double CommonControl::Stand_base_y(double y)
        {
                double y1 = y - M_PI / 2;
                if (y1 > M_PI)
                {
                        y1 -= 2 * M_PI;
                }
                else if (y1 < -M_PI)
                {
                        y1 += 2 * M_PI;
                }
                return y1;
        }

        void CommonControl::Stop()
        {
                geometry_msgs::Twist vel;
                vel.linear.x = 0;
                vel.angular.z = 0;
                odom1::underpan_speed vel_msg;
                vel_msg.left_speed = (2 * vel.linear.x - B * vel.angular.z) * (1 - fl) / 2;    // 左轮速度
                vel_msg.right_speed = (2 * vel.linear.x + B * vel.angular.z) / ((1 - fr) * 2); // 右轮速度
                vel_pub.publish(vel_msg);
        }

        bool CommonControl::task_srv(project_truck::hmh_task::Request &req, project_truck::hmh_task::Response &resp)
        {
                ROS_INFO("task_srv called with type: %d, command: %d", req.type, req.command);

                switch (req.type)
                {
                case project_truck::hmh_task::Request::EXECUTE:
                        switch (req.command)
                        {
                        case project_truck::hmh_task::Request::START:
                                if (Robot_State == StateValue::Pause)
                                {
                                        Robot_State = StateValue::Run;
                                        Run_State = RunStateValue::Goto;
                                        ROS_INFO("Task resumed. State changed to: %d", static_cast<int>(Robot_State));
                                }
                                else if (Robot_State == StateValue::Idle)
                                {
                                        Robot_State = StateValue::Run;
                                        Run_State = RunStateValue::Goto;
                                        ROS_INFO("Started new task. State changed to: %d", static_cast<int>(Robot_State));
                                }
                                else
                                {
                                        ROS_WARN("Already running a task, ignoring start command.");
                                }
                                break;

                        case project_truck::hmh_task::Request::PAUSE:
                                if (Robot_State == StateValue::Run)
                                {
                                        Robot_State = StateValue::Pause;
                                        ROS_INFO("Task paused. State changed to: %d", static_cast<int>(Robot_State));
                                }
                                else
                                {
                                        ROS_WARN("Not in running state, ignoring pause command.");
                                }
                                break;

                        case project_truck::hmh_task::Request::STOP:
                                if (Robot_State == StateValue::Run || Robot_State == StateValue::Pause)
                                {
                                        Robot_State = StateValue::Idle;
                                        Run_State = RunStateValue::Goto;
                                        Stop();
                                        ROS_INFO("Task stopped. State changed to: %d", static_cast<int>(Robot_State));
                                }
                                else
                                {
                                        ROS_WARN("Not in running or pause state, ignoring stop command.");
                                }
                                break;

                        default:
                                ROS_WARN("Unknown command %d for EXECUTE type, ignoring.", req.command);
                                break;
                        }
                        break;
                case project_truck::hmh_task::Request::RECORD:
                        switch (req.command)
                        {
                        case project_truck::hmh_task::Request::START:
                                if (Robot_State == StateValue::Idle)
                                {
                                        Robot_State = StateValue::Record;
                                        ROS_INFO("Started recording path. State changed to: %d", static_cast<int>(Robot_State));
                                }
                                else
                                {
                                        ROS_WARN("Not in idle state, ignoring record command.");
                                }
                                break;

                        default:
                                ROS_WARN("Unknown command %d for RECORD type, ignoring.", req.command);
                                break;
                        }
                        break;

                default:
                        ROS_WARN("Unknown type %d, ignoring.", req.type);
                        break;
                }

                resp.message = "State transitioned.";
                return true;
        }
}