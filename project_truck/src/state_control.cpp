#include <chrono>
#include <fstream>
#include <cmath>
#include"state_control.h"


namespace State_Control
{
        StateControl::StateControl() : 
        robot_type(false),
        // robot_type(true),
        Robot_State(StateValue::Idle),  // 车的状态
        Run_State(RunStateValue::Goto), // 车的运行状态
        current_point(),
        current_index(),
        goal_point(),
        odom_callback_flag(false)
        {
                path_vector = ReadFile(PATH_NAME);
        }
        StateControl::~StateControl()
        {
                Stop();
        }

        void StateControl::Init()
        {
                ros::NodeHandle nh;
                if (robot_type)
                {
                        vel_pub_ = nh.advertise<yhs_can_msgs::ctrl_cmd>(VELPUB_TRUE, 1);
                }
                else
                {
                        vel_pub_ = nh.advertise<geometry_msgs::Twist>(VELPUB_SIM, 1);
                        robot_1_vel_pub_ = nh.advertise<geometry_msgs::Twist>(Robot_1_vel_topic, 1);
                        robot_2_vel_pub_ = nh.advertise<geometry_msgs::Twist>(Robot_2_vel_topic, 1);
                }
                path_pub_ = nh.advertise<nav_msgs::Path>("pub_path", 1);
                marker_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
                robot_path_pub_ = nh.advertise<nav_msgs::Path>("robot_path", 1);
                if (robot_type)
                {
                        odom_sub_ = nh.subscribe(Odom_TRUE, 1, &StateControl::OdomCallback_true, this);
                }
                else
                {
                        odom_sub_ = nh.subscribe(Odom_SIM, 1, &StateControl::OdomCallback_sim, this);
                        robot_1_odom_sub_ = nh.subscribe(Robot_1_odom_topic, 1, &StateControl::OdomCallback_sim, this);
                        robot_2_odom_sub_ = nh.subscribe(Robot_2_odom_topic, 1, &StateControl::OdomCallback_sim, this);
                }
                task_srv_ = nh.advertiseService(TASK_SRV, &StateControl::task_srv, this);

                ROS_INFO("hmh_state_control initialized!");
        }

        void StateControl::Run()
        {
                ros::Rate rate(100);

                while (ros::ok())
                {
                        if (path_vector.empty())
                        {
                                ROS_WARN("Path vector is empty. Exiting.");
                                break;
                        }
                        ros::spinOnce();
                        ROS_INFO("Robot_State: %d", static_cast<int>(Robot_State));
                        // ROS_INFO("Robot_1_Pose: %f, %f, %f", robot_1_pose.x, robot_1_pose.y, robot_1_pose.yaw);
                        // ROS_INFO("Robot_2_Pose: %f, %f, %f", robot_2_pose.x, robot_2_pose.y, robot_2_pose.yaw);
                        StateControl::Visual_Path(path_vector);
                        switch (Robot_State)
                        {
                        case StateValue::Idle:
                                ROS_INFO("Robot is Idle!");

                                break;
                        case StateValue::Pause:
                                ROS_INFO("Robot is Pause!");

                                break;
                        case StateValue::Record:
                                ROS_INFO("Robot is Record!");

                                break;
                        case StateValue::Run:
                                Running_State();
                                break;  

                        default:
                                ROS_ERROR("Error StateValue!");
                                break;
                        }
                        // ros::spinOnce();
                        rate.sleep();
                }
        }

        void StateControl::Running_State()
        {
                switch (Run_State)
                {
                case RunStateValue::Goto:
                {
                        ROS_INFO("Goto");
                        Goto_Goal(path_vector);
                        break;
                }

                case RunStateValue::Tracking:
                        ROS_INFO("Tracking");
                        Tracking_Path();
                        break;

                case RunStateValue::Follow:
                        ROS_INFO("Follow");
                        break;

                case RunStateValue::Wait:
                        ROS_INFO("Wait");
                        break;

                default:
                        ROS_INFO("Error RunStateValue!");
                        break;
                }
        }

        void StateControl::Stop()
        {
                if (robot_type)
                {
                        yhs_can_msgs::ctrl_cmd vel_msg;
                        vel_msg.ctrl_cmd_gear = 06;
                        vel_msg.ctrl_cmd_x_linear = 0.0;
                        vel_msg.ctrl_cmd_y_linear = 0;
                        vel_msg.ctrl_cmd_z_angular = 0.0;
                        vel_pub_.publish(vel_msg);
                }
                else
                {
                        geometry_msgs::Twist vel_msg;
                        vel_msg.angular.z = 0;
                        vel_msg.linear.x = 0.0;
                        vel_pub_.publish(vel_msg);
                }
        }

        void StateControl::Goto_Goal(Path_Type path_point)
        {
                if(path_point.empty()) return;
                
                double target_x = path_point[0].first.first;
                double target_y = path_point[0].first.second;
                double target_yaw = path_point[0].second;
                // ros::Rate rate(100);

                ros::spinOnce();
                ROS_INFO("Goto_Goal: %f, %f, %f", target_x, target_y, target_yaw);
                ROS_INFO("Robot_Pose: %f, %f, %f", robot_pose.x, robot_pose.y, robot_pose.yaw);

                double distance = sqrt(pow(target_x - robot_pose.x, 2) + pow(target_y - robot_pose.y, 2));
                ROS_INFO("distance: %f", distance);
                if (distance >= 0.05)
                {
                        if (robot_type)
                        {
                                double angle_to_goal = atan2(target_x - robot_pose.x, target_y - robot_pose.y);

                                yhs_can_msgs::ctrl_cmd vel_msg;
                                vel_msg.ctrl_cmd_gear = 06;
                                vel_msg.ctrl_cmd_x_linear = goal_speed;
                                vel_msg.ctrl_cmd_y_linear = 0;
                                vel_msg.ctrl_cmd_z_angular = kd_g * (angle_to_goal - robot_pose.yaw) * 180 / M_PI;
                                vel_pub_.publish(vel_msg);
                        }
                        else
                        {
                                double angle_to_goal = atan2(target_y - robot_pose.y, target_x - robot_pose.x);

                                geometry_msgs::Twist vel_msg;
                                vel_msg.angular.z = 1 * (angle_to_goal - robot_pose.yaw);
                                vel_msg.linear.x = 0.5;

                                vel_pub_.publish(vel_msg);
                        }
                }
                else
                {
                        ROS_INFO("yaw_err: %f", std::abs(target_yaw - robot_pose.yaw));
                        if (std::abs(target_yaw - robot_pose.yaw )> 0.05)
                        {
                                if (robot_type)
                                {
                                        yhs_can_msgs::ctrl_cmd vel_msg;
                                        vel_msg.ctrl_cmd_gear = 06;
                                        vel_msg.ctrl_cmd_x_linear = 0.0;
                                        vel_msg.ctrl_cmd_y_linear = 0;
                                        vel_msg.ctrl_cmd_z_angular = -0.1*180/M_PI;
                                        vel_pub_.publish(vel_msg);
                                }
                                else
                                {
                                        geometry_msgs::Twist vel_msg;
                                        vel_msg.angular.z =0.5;
                                        vel_msg.linear.x = 0.0;
                                        vel_pub_.publish(vel_msg);
                                }
                        }
                        else
                        {
                                if (robot_type)
                                {
                                        yhs_can_msgs::ctrl_cmd vel_msg;
                                        vel_msg.ctrl_cmd_gear = 06;
                                        vel_msg.ctrl_cmd_x_linear = 0.0;
                                        vel_msg.ctrl_cmd_y_linear = 0;
                                        vel_msg.ctrl_cmd_z_angular = 0.0;
                                        vel_pub_.publish(vel_msg);
                                }
                                else
                                {
                                        geometry_msgs::Twist vel_msg;
                                        vel_msg.angular.z = 0;
                                        vel_msg.linear.x = 0.0;
                                        vel_pub_.publish(vel_msg);
                                }

                                ROS_INFO("goto goal success");
                                Run_State = RunStateValue::Tracking;
                        }

                }
                // rate.sleep();
        }
        void StateControl::Tracking_Path()
        {

                current_point.first.first=robot_pose.x;
                current_point.first.second=robot_pose.y;
                current_point.second=robot_pose.yaw;

                current_index = GetMinDistance(current_point, path_vector);
                goal_point = path_vector[current_index];
                ROS_INFO("Goal_Point: (X=%f,Y= %f,YAW= %f)",goal_point.first.first, goal_point.first.second, goal_point.second);
                ROS_INFO("Robot_Pose: (X=%f,Y= %f,YAW= %f)", robot_pose.x, robot_pose.y, robot_pose.yaw);
                if (current_index == path_vector.size() - 1)
                {
                        ROS_INFO("Reached the last point. Stopping the robot.");
                        Run_State = RunStateValue::Follow;
                }

                double e_y = Get_Distance(current_point, goal_point);
                double delta = 0.0;
                if (robot_type)
                {
                        e_y = ((current_point.first.second - goal_point.first.second) *
                                   sin(goal_point.second) -
                               (current_point.first.first - goal_point.first.first) *
                                   cos(goal_point.second)) <= 0
                                  ? e_y
                                  : -e_y;
                        double theta_e = goal_point.second - current_point.second;
                        double delta_e = atan2(kp * e_y, truck_speed);
                        delta = delta_e + theta_e;
                }
                else
                {
                        e_y = ((current_point.first.second - goal_point.first.second) *
                                   cos(goal_point.second) -
                               (current_point.first.first - goal_point.first.first) *
                                   sin(goal_point.second)) <= 0
                                  ? e_y
                                  : -e_y;
                        double theta_e = goal_point.second - current_point.second;
                        double delta_e = atan2(kp * e_y, truck_speed);
                        delta = delta_e + theta_e;
                }

                if (delta > M_PI)
                {
                        delta -= 2 * M_PI;
                }
                else if (delta < -M_PI)
                {
                        delta += 2 * M_PI;
                }
                if (robot_type)
                {
                        yhs_can_msgs::ctrl_cmd vel_msg;
                        vel_msg.ctrl_cmd_gear = 06;
                        vel_msg.ctrl_cmd_x_linear = truck_speed;
                        vel_msg.ctrl_cmd_y_linear = 0;
                        vel_msg.ctrl_cmd_z_angular = delta * 180 / M_PI;
                        if (odom_callback_flag)
                        {
                                vel_pub_.publish(vel_msg);
                        }
                }
                else
                {
                        ROS_INFO("delta: %f", delta);
                        geometry_msgs::Twist vel_msg;
                        vel_msg.linear.x = 0.1;
                        vel_msg.angular.z = delta;
                        if (odom_callback_flag)
                        {
                                vel_pub_.publish(vel_msg);
                        }
                }

        }

        Path_Type StateControl::ReadFile(const std::string &filePath)
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
                        double yaw = std::stod(tokens[3])/180.0*M_PI;
                        // double yaw = std::stod(tokens[6]);
                        path_vector.push_back({{x, y}, yaw-M_PI/2});
                }
                return path_vector;
        }
        void StateControl::Visual_Path(Path_Type path)
        {
                visualization_msgs::Marker marker;
                marker.header.frame_id = "odom";
                marker.header.stamp = ros::Time::now();
                marker.ns = "odom";
                marker.id = 0;
                marker.type = visualization_msgs::Marker::LINE_STRIP;
                marker.action = visualization_msgs::Marker::ADD;
                marker.lifetime = ros::Duration();
                marker.color.b = 1.0;
                marker.color.a = 1.0;
                marker.scale.x = 0.02;
                marker.pose.orientation.w = 1.0;

                for (auto &point : path)
                {
                        geometry_msgs::Point p;
                        p.x = point.first.first;
                        p.y = point.first.second;
                        p.z = 0;
                        marker.points.push_back(p);
                }
                marker_pub_.publish(marker);
        }

        void StateControl::Pub_Robot_Path(const nav_msgs::Odometry::ConstPtr &odom)
        {
                geometry_msgs::PoseStamped robot_pose_stamped;
                robot_pose_stamped.pose.position.x = odom->pose.pose.position.x;
                robot_pose_stamped.pose.position.y = odom->pose.pose.position.y;
                robot_pose_stamped.pose.position.z = odom->pose.pose.position.z;
                robot_pose_stamped.pose.orientation = odom->pose.pose.orientation;
                robot_pose_stamped.header.stamp = ros::Time::now();
                robot_pose_stamped.header.frame_id = "odom";
                path.poses.push_back(robot_pose_stamped);
                path.header.stamp = ros::Time::now();
                path.header.frame_id = "odom";
                path_pub_.publish(path);
        }

        Path_Type StateControl::GetFirstPoint(Path_Type path)
        {
                Path_Type first_point;
                first_point.push_back(path[0]);
                return first_point;
        }

        double StateControl::Get_Distance(path_type point1, path_type point2)
        {
                double dis = sqrt(pow(point1.first.first - point2.first.first, 2) + pow(point1.first.second - point2.first.second, 2));
                return dis;
        }

        int StateControl::GetMinDistance(path_type current_point, Path_Type path_vector)
        {
                double min_dis = INFINITY;
                int index = 0;
                for (int i = 0; i < path_vector.size(); i++)
                {
                        double distance = StateControl::Get_Distance(path_vector[i], current_point);
                        if (distance < min_dis)
                        {
                                min_dis = distance;
                                index = i;
                        }
                }
                return index;
        }

        void StateControl::OdomCallback_sim(const nav_msgs::Odometry::ConstPtr &msg)
        {
                odom_callback_flag = true;
                robot_pose.x = msg->pose.pose.position.x;
                robot_pose.y = msg->pose.pose.position.y;
                robot_pose.yaw = tf::getYaw(msg->pose.pose.orientation);
                if (msg->header.frame_id == Robot_1_odom_topic)
                {
                        robot_1_pose = robot_pose;
                }
                else if (msg->header.frame_id == Robot_2_odom_topic)
                {
                        robot_2_pose = robot_pose;
                }
                Pub_Robot_Path(msg);
        }
        void StateControl::OdomCallback_true(const geometry_msgs::PoseStamped::ConstPtr &msg)
        {
                odom_callback_flag = true;
                robot_pose.x = msg->pose.position.x;
                robot_pose.y = msg->pose.position.y;
                robot_pose.yaw = - msg->pose.orientation.z;
        }

        bool StateControl::task_srv(project_truck::hmh_task::Request &req, project_truck::hmh_task::Response &resp)
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
