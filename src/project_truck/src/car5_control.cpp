#include "project_truck/common_control.h" // 引用公共代码库
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include "std_msgs/Float64.h"

namespace Car5_Control
{
        class Car5Control : public Common_Control::CommonControl
        {
        public:
                Car5Control(const std::string &robot_name, const std::string &odom_topic, const std::string &robot_topic, const std::string &pose_pub_topic)
                    : CommonControl(robot_name, odom_topic, robot_topic, pose_pub_topic)
                {
                        car1_state = Common_Control::StateValue::Idle; // 初始化机器人1的状态为Idle
                }
                void Init()
                {
                        CommonControl::Init();
                        ros::NodeHandle nh;
                        car4_pose_sub = nh.subscribe("car4_pose", 1, &Car5Control::car4PoseCallback, this);
                        car1_state_sub = nh.subscribe("car1_state", 1, &Car5Control::car1StateCallback, this); // 订阅机器人1的状态
                        marker1_pub = nh.advertise<visualization_msgs::Marker>("visualization4_marker", 1);
                        dis_pub=nh.advertise<std_msgs::Float64>("dis_car4_car5",1);
                }

                void Run()
                {
                        ros::Rate rate(100); // 设置循环频率
                        while (ros::ok())
                        {
                                ros::spinOnce(); // 处理ROS回调

                                if (car4_path_vector.empty())
                                {

                                        ROS_INFO("car4_path_vector is empty,waiting");
                                        ROS_INFO("car2_pose:%f,%f,%f,", robot_pose.x, robot_pose.y, robot_pose.yaw);
                                        rate.sleep();
                                        continue;
                                }
                                dis_car4_car5 = dis;
                                std_msgs::Float64 dis_msg;
                                dis_msg.data=dis_car4_car5;
                                dis_pub.publish(dis_msg);
                                ROS_INFO("dis_car4_car5=%f",dis_car4_car5);
                                // if (car1_state != Common_Control::StateValue::Run)
                                // {
                                //         ROS_INFO("car1 is not in Run state. Robot2 is waiting...");
                                //         rate.sleep();
                                //         continue;
                                // }
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
                ros::Subscriber car4_pose_sub; // 订阅机器人1的路径点话题
                ros::Subscriber car1_state_sub; // 订阅机器人1的状态话题
                ros::Publisher dis_pub;
                Path_Type car4_path_vector;    // 存储机器人1的路径点
                ros::Publisher marker1_pub;
                double dis;
                double dis_car4_car5;
                Common_Control::StateValue car1_state; // 机器人1的状态

                void Running_State()
                {
                        // 获取当前目标路径点
                        path_type current_goal = car4_path_vector[current_index];

                        // 检查运行状态
                        switch (Run_State)
                        {
                        case Common_Control::RunStateValue::Goto:
                                ROS_INFO("Robot2 is going to the first goal point.");
                                // 前往第一个路径点
                                /*
                                        Goto_Goal 第一个参数为终点坐标，第二个参数为距离目标点最小距离，第三个参数为速度，第四个参数为角速度调整参数,第五个参数判断是不是goto状态
                                */

                                Goto_Goal(car4_path_vector[current_index], 1, 0.02 + 1.0 * (dis - 9.0), 0.002, 1.0);
                                
                                break;

                        case Common_Control::RunStateValue::Tracking:
                                ROS_INFO("Robot2 is tracking the path.");
                                // 轨迹跟踪
                                /*
                                        Tracking_Path_pure_pursuit 第一个参数为路径点，第二个参数为前瞻距离，第三个参数为速度，第四个为角速度调整参数
                                */
                                if (current_index < car4_path_vector.size())
                                {
                                        Tracking_Path_pure_pursuit(car4_path_vector, 1, 0.02 + 1 * (dis - 9.0), 0.08);
                                }
                                else
                                {
                                        ROS_INFO("Robot2 has completed tracking the path.");
                                        Robot_State = Common_Control::StateValue::Idle;
                                }
                                break;
                        case Common_Control::RunStateValue::Stop:
                                ROS_INFO("Robot2 is stopping.");
                                Stop(); // 停止机器人
                                break;

                        default:
                                ROS_ERROR("Unknown Run State!");
                                break;
                        }
                }

                void Tracking_Path_pure_pursuit(Path_Type path_vector, double ld, double speed, double angle_gain)
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

                        ROS_INFO("goal_point:x:=%f,y:=%f,yaw:=%f", ahead_path_point.first.first, ahead_path_point.first.second, ahead_path_point.second);
                        ROS_INFO("current_point:x:=%f,y:=%f,yaw:=%f", current_point.first.first, current_point.first.second, current_point.second);
                        // 计算转向角度
                        double come_goal_yaw = CommonControl::Stand_base_y(std::atan2(ahead_path_point.first.second - current_point.first.second, ahead_path_point.first.first - current_point.first.first));
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

                        // 发布速度命令
                        geometry_msgs::Twist vel;
                        vel.linear.x = speed ;
                        vel.angular.z = angle_gain * delta;
                        odom1::underpan_speed vel_msg;
                        vel_msg.left_speed = (2 * vel.linear.x - B * vel.angular.z) * (1 - fl) / 2;    // 左轮速度
                        vel_msg.right_speed = (2 * vel.linear.x + B * vel.angular.z) / ((1 - fr) * 2); // 右轮速度
                        if (odom_callback_flag)
                        {
                                vel_pub.publish(vel_msg);
                        }
                }

                void car4PoseCallback(const project_truck::RobotPose::ConstPtr &msg)
                {
                        // 将接收到的路径点添加到路径向量中
                        car4_path_vector.push_back({{msg->x, msg->y}, msg->yaw});
                        dis = sqrt(pow(msg->x - robot_pose.x, 2) + pow(msg->y - robot_pose.y, 2));
                        // 可视化机器人1的路径点
                        Visual_Path(car4_path_vector);
                }
                void car1StateCallback(const std_msgs::String::ConstPtr &msg)
                {
                        // 更新机器人1的状态
                        if (msg->data == "Run")
                        {
                                car1_state = Common_Control::StateValue::Run;
                        }
                        else if (msg->data == "Idle")
                        {
                                car1_state = Common_Control::StateValue::Idle;
                        }
                        else if (msg->data == "Pause")
                        {
                                car1_state = Common_Control::StateValue::Pause;
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
                        marker.color.r = 1.0;
                        marker.color.b = 0.0;
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
                        marker1_pub.publish(marker);
                }
        };
}

int main(int argc, char *argv[])
{
        ros::init(argc, argv, "car5_control");

        std::string robot_name = "car5";
        std::string odom_topic = "/current_pose";
        std::string robot_topic = "/underpan_speed_1";
        std::string pose_pub_topic = "car5_pose";

        Car5_Control::Car5Control car2(robot_name, odom_topic, robot_topic, pose_pub_topic);
        car2.Init(); // 初始化
        car2.Run();  // 运行

        return 0;
}