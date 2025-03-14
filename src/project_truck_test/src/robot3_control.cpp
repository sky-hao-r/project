#include "common_control.h" // 引用公共代码库
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

namespace Robot3_Control
{
        class Robot3Control : public Common_Control::CommonControl
        {
        public:
                Robot3Control(const std::string &robot_name, const std::string &odom_topic, const std::string &robot_topic, const std::string &pose_pub_topic)
                    : CommonControl(robot_name, odom_topic, robot_topic, pose_pub_topic)
                {
                        robot_type = false;
                        robot1_state = Common_Control::StateValue::Idle; // 初始化机器人1的状态为Idle
                }
                void Init()
                {
                        CommonControl::Init();
                        ros::NodeHandle nh;

                        robot2_pose_sub = nh.subscribe("robot2_pose", 1, &Robot3Control::Robot2PoseCallback, this);
                        robot1_state_sub = nh.subscribe("robot1_state", 1, &Robot3Control::Robot1StateCallback, this); // 订阅机器人1的状态
                        marker2_pub = nh.advertise<visualization_msgs::Marker>("visualization2_marker", 1);
                }

                void Run()
                {
                        ros::Rate rate(100); // 设置循环频率
                        while (ros::ok())
                        {
                                ros::spinOnce(); // 处理ROS回调
                                if (robot2_path_vector.empty())
                                {
                                        ROS_INFO("robot2_path_vector is empty,waiting");
                                        rate.sleep();
                                        continue;
                                }
                                if (robot1_state != Common_Control::StateValue::Run)
                                {
                                        ROS_INFO("Robot1 is not in Run state. Robot2 is waiting...");
                                        rate.sleep();
                                        continue;
                                }
                                switch (Robot_State)
                                {
                                case Common_Control::StateValue::Idle:
                                        // 开始运行前的准备
                                        Robot_State = Common_Control::StateValue::Run;
                                        current_index = 0; // 初始化当前路径点索引
                                        ROS_INFO("Robot3 is transitioning to Run state...");
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
                ros::Subscriber robot2_pose_sub; // 订阅机器人2的路径点话题
                ros::Subscriber robot1_state_sub; // 订阅机器人1的状态话题

                Path_Type robot2_path_vector;    // 存储机器人2的路径点
                ros::Publisher marker2_pub;
                double dis;
                Common_Control::StateValue robot1_state; // 机器人1的状态

                void Running_State()
                {
                        // 获取当前目标路径点
                        path_type current_goal = robot2_path_vector[current_index];

                        // 检查运行状态
                        switch (Run_State)
                        {
                        case Common_Control::RunStateValue::Goto:
                                ROS_INFO("Robot3 is going to the first goal point.");
                                // 前往第一个路径点
                                if (robot_type)
                                {
                                        Goto_Goal(robot2_path_vector[current_index], 0.05, 0.3 + 1.0 * (dis - 2.0), 1.0, 1.0);
                                }
                                else
                                {
                                        Goto_Goal(robot2_path_vector[current_index], 0.05, 0.3 + 1.0 * (dis - 2.0), 1.0, 1.0);
                                }
                                
                                break;

                        case Common_Control::RunStateValue::Tracking:
                                ROS_INFO("Robot2 is tracking the path.");
                                // 轨迹跟踪
                                if (current_index < robot2_path_vector.size())
                                {
                                        // Tracking_Path_stanley(robot1_path_vector);
                                        Tracking_Path_pure_pursuit(robot2_path_vector,0.05,0.1);
                                }
                                else
                                {
                                        ROS_INFO("Robot3 has completed tracking the path.");
                                        Robot_State = Common_Control::StateValue::Idle;
                                }
                                break;
                        case Common_Control::RunStateValue::Stop:
                                ROS_INFO("Robot3 is stopping.");
                                Stop(); // 停止机器人
                                break;

                        default:
                                ROS_ERROR("Unknown Run State!");
                                break;
                        }
                }

                void Tracking_Path_stanley(Path_Type path)
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
                                Run_State = Common_Control::RunStateValue::Stop;
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
                        double delta = delta_e + 0.5 * theta_e;

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
                                vel_msg.ctrl_cmd_x_linear = truck_speed+1.0*(dis-2.0);
                                vel_msg.ctrl_cmd_z_angular = delta * 180 / M_PI;
                                if (odom_callback_flag)
                                {
                                        vel_pub.publish(vel_msg);
                                }
                        }
                        else
                        {
                                geometry_msgs::Twist vel_msg;
                                vel_msg.linear.x = truck_speed + 1.0 * (dis - 2.0);
                                vel_msg.angular.z = delta;
                                if (odom_callback_flag)
                                {
                                        vel_pub.publish(vel_msg);
                                }
                        }

                        ROS_INFO("delta: %f", delta);
                }

                void Tracking_Path_pure_pursuit(Path_Type path_vector, double ld, double speed)
                {
                        // 获取当前位置与目标路径点
                        path_type current_point = {{robot_pose.x, robot_pose.y}, robot_pose.yaw};
                        int nearest_index = GetMinDistance(current_point, path_vector); // 找到最近的路径点索引

                        if (nearest_index > current_index)
                        {
                                current_index = nearest_index; // 更新为最近的路径点索引
                        }

                        // 目标点
                        path_type goal_point = path_vector[current_index];
                        ROS_INFO("nearest_index: %d", nearest_index);
                        ROS_INFO("current_index: %d", current_index);

                        // 检查是否到达终点
                        if (current_index == path_vector.size() - 1)
                        {
                                ROS_INFO("Reached the last point. Stopping the robot.");
                                goto_complete_flag = false;
                                Run_State = Common_Control::RunStateValue::Stop;
                        }

                        // 从 current_index 开始搜索满足条件的路径点
                        int start_index = current_index;
                        path_type ahead_path_point;
                        while (true)
                        {
                                // 获取当前路径点
                                ahead_path_point = path_vector[current_index];

                                // 计算超前点与当前路径点的距离
                                double dis1 = Get_Distance(current_point, ahead_path_point);
                                if (dis1 >= ld)
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

                        // 计算转向角度
                        double alpha = std::atan2(ahead_path_point.first.second - current_point.first.second,
                                                  ahead_path_point.first.first - current_point.first.first) -
                                       current_point.second;

                        double delta = std::atan2(2 * speed * sin(alpha), ld);

                        if (delta > M_PI)
                        {
                                delta -= 2 * M_PI;
                        }
                        else if (delta < -M_PI)
                        {
                                delta += 2 * M_PI;
                        }

                        // 发布速度命令
                        if (robot_type)
                        {
                                yhs_can_msgs::ctrl_cmd vel_msg;
                                vel_msg.ctrl_cmd_gear = 06;
                                vel_msg.ctrl_cmd_x_linear = speed + 1.0 * (dis - 2.0);
                                vel_msg.ctrl_cmd_y_linear = 0;
                                vel_msg.ctrl_cmd_z_angular = delta * 180 / M_PI;
                                if (odom_callback_flag)
                                {
                                        vel_pub.publish(vel_msg);
                                }
                        }
                        else
                        {
                                geometry_msgs::Twist vel_msg;
                                vel_msg.linear.x = speed + 1.0 * (dis - 2.0);
                                vel_msg.angular.z = delta;
                                if (odom_callback_flag)
                                {
                                        vel_pub.publish(vel_msg);
                                }
                        }
                }

                void Robot2PoseCallback(const project_truck::RobotPose::ConstPtr &msg)
                {
                        // 将接收到的路径点添加到路径向量中
                        robot2_path_vector.push_back({{msg->x, msg->y}, msg->yaw});
                        dis = sqrt(pow(msg->x - robot_pose.x, 2) + pow(msg->y - robot_pose.y, 2));
                        // 可视化机器人1的路径点
                        Visual_Path(robot2_path_vector);
                }
                void Robot1StateCallback(const std_msgs::String::ConstPtr &msg)
                {
                        // 更新机器人1的状态
                        if (msg->data == "Run")
                        {
                                robot1_state = Common_Control::StateValue::Run;
                        }
                        else if (msg->data == "Idle")
                        {
                                robot1_state = Common_Control::StateValue::Idle;
                        }
                        else if (msg->data == "Pause")
                        {
                                robot1_state = Common_Control::StateValue::Pause;
                        }
                }

                void Visual_Path(const Path_Type &path)
                {
                        visualization_msgs::Marker marker;
                        marker.header.frame_id = "world";
                        marker.header.stamp = ros::Time::now();
                        marker.ns = robot_name;
                        marker.id = 0;
                        marker.type = visualization_msgs::Marker::LINE_STRIP;
                        marker.action = visualization_msgs::Marker::ADD;
                        marker.lifetime = ros::Duration();
                        marker.color.r = 0.0;
                        marker.color.b = 1.0;
                        marker.color.g = 0.0;
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
                        marker2_pub.publish(marker);
                }
        };
}

int main(int argc, char *argv[])
{
        ros::init(argc, argv, "robot3_control");

        std::string robot_name = "robot3";
        std::string odom_topic = "/robot3/current_pose";
        std::string robot_topic = "/robot3/cmd_vel";
        std::string pose_pub_topic = "robot3_pose";

        Robot3_Control::Robot3Control robot3(robot_name, odom_topic, robot_topic, pose_pub_topic);
        robot3.Init(); // 初始化
        robot3.Run();  // 运行

        return 0;
}