#include "common_control.h"
#include <chrono>
#include <fstream>

namespace Common_Control
{
        CommonControl::CommonControl(const std::string &robot_name, const std::string &odom_topic, const std::string &robot_topic, const std::string &pose_pub_topic)
            : robot_name(robot_name), odom_topic(odom_topic), robot_topic(robot_topic), pose_pub_topic(pose_pub_topic), odom_callback_flag(false), goto_complete_flag(false),Robot_State(StateValue::Idle), Run_State(RunStateValue::Goto) // 车的运行状态
        {
        }

        CommonControl::~CommonControl()
        {

        }

        void CommonControl::Init()
        {
                ros::NodeHandle nh;

                odom_sub = nh.subscribe(odom_topic, 1, &CommonControl::OdomCallback, this);
                vel_pub = nh.advertise<geometry_msgs::Twist>(robot_topic, 1);
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
                        // Parse the coordinates and quaternion
                        double x = std::stod(tokens[0]);
                        double y = std::stod(tokens[1]);
                        double z = std::stod(tokens[2]);
                        // double x = std::stod(tokens[1]);
                        // double y = std::stod(tokens[2]);
                        // double z = std::stod(tokens[3]);
                        // double qx = std::stod(tokens[4]);
                        // double qy = std::stod(tokens[5]);
                        // double qz = std::stod(tokens[6]);
                        // double qw = std::stod(tokens[7]);
                        geometry_msgs::Point path_point;
                        path_point.x = x;
                        path_point.y = y;
                        path_point.z = 0;
                        // Convert quaternion to yaw angle
                        // tf::Quaternion q(qx, qy, qz, qw);
                        double yaw = std::stod(tokens[3]) / 180.0 * M_PI;
                        // double yaw = std::stod(tokens[6]);
                        path_vector.push_back({{x, y}, yaw - M_PI / 2});
                }
                return path_vector;
        }


        void CommonControl::Visual_Path(const Path_Type &path)
        {
                visualization_msgs::Marker marker;
                marker.header.frame_id = "odom";
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

        void CommonControl::OdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
        {
                odom_callback_flag = true;
                robot_pose.x = msg->pose.pose.position.x;
                robot_pose.y = msg->pose.pose.position.y;
                robot_pose.yaw = tf::getYaw(msg->pose.pose.orientation);

                robot_pose_msg.x = robot_pose.x;
                robot_pose_msg.y = robot_pose.y;
                robot_pose_msg.yaw = robot_pose.yaw;
                if(goto_complete_flag)
                {
                        pose_pub.publish(robot_pose_msg);
                }
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

        void CommonControl::Stop()
        {
                geometry_msgs::Twist vel_msg;
                vel_msg.linear.x = 0.0;
                vel_msg.angular.z = 0.0;
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