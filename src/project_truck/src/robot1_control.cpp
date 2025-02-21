#include "common_control.h" // 引用公共代码库

namespace Robot1_Control
{
        class Robot1Control : public Common_Control::CommonControl
        {
        public:
                Robot1Control(const std::string &robot_name, const std::string &odom_topic, const std::string &robot_topic, const std::string &pose_pub_topic)
                    : CommonControl(robot_name, odom_topic, robot_topic, pose_pub_topic)
                {
                        path_vector = ReadFile("/home/hmh/project_truck/src/project_truck/Data/input_1.csv");
                        watting_point={{34.0,10.0},M_PI/2};
                }

                void Run()
                {
                        ros::Rate rate(100); // 设置循环频率

                        while (ros::ok())
                        {
                                if (path_vector.empty())
                                {
                                        ROS_WARN("Path vector is empty. Exiting.");
                                        break;
                                }
                                ros::spinOnce(); // 处理ROS回调

                                ROS_INFO("Robot_State: %d", static_cast<int>(Robot_State));
                                CommonControl::Visual_Path(path_vector);
                                switch (Robot_State)
                                {
                                case Common_Control::StateValue::Idle:
                                        ROS_INFO("Robot is Idle!");
                                        break;

                                case Common_Control::StateValue::Pause:
                                        ROS_INFO("Robot is Paused!");
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
                path_type watting_point;

                void Running_State()
                {
                        switch (Run_State)
                        {
                        case Common_Control::RunStateValue::Goto:
                                Goto_Goal(path_vector);
                                break;

                        case Common_Control::RunStateValue::Tracking:
                                Tracking_Path();
                                break;
                        case Common_Control::RunStateValue::Waitting:
                                Goto_Watting_Point(watting_point);
                                break;
                        case Common_Control::RunStateValue::Stop:
                                Stop();
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
                                if(robot_type)
                                {
                                        double angle_to_goal = atan2(target_x - robot_pose.x, target_y - robot_pose.y);

                                        yhs_can_msgs::ctrl_cmd vel_msg;
                                        vel_msg.ctrl_cmd_gear = 06;
                                        vel_msg.ctrl_cmd_x_linear = 0.3;
                                        vel_msg.ctrl_cmd_y_linear = 0;
                                        vel_msg.ctrl_cmd_z_angular = 0.4 * (angle_to_goal - robot_pose.yaw) * 180 / M_PI;
                                        vel_pub.publish(vel_msg);
                                }
                                else
                                {
                                        double angle_to_goal = atan2(target_y - robot_pose.y, target_x - robot_pose.x);

                                        geometry_msgs::Twist vel_msg;
                                        vel_msg.angular.z = 4 * (angle_to_goal - robot_pose.yaw);
                                        vel_msg.linear.x = 0.1;
                                        ROS_INFO("w: %f", angle_to_goal - robot_pose.yaw);
                                        vel_pub.publish(vel_msg);
                                }
                        }
                        else
                        {
                                ROS_INFO("yaw_err: %f", std::abs(target_yaw - robot_pose.yaw));
                                if (std::abs(target_yaw - robot_pose.yaw) > 0.05)
                                {
                                        if (robot_type)
                                        {
                                                yhs_can_msgs::ctrl_cmd vel_msg;
                                                vel_msg.ctrl_cmd_gear = 06;
                                                vel_msg.ctrl_cmd_x_linear = 0.0;
                                                vel_msg.ctrl_cmd_y_linear = 0;
                                                if (target_yaw - robot_pose.yaw > 0)
                                                {
                                                        vel_msg.ctrl_cmd_z_angular = 0.1 * 180 / M_PI;
                                                }
                                                else
                                                {
                                                        vel_msg.ctrl_cmd_z_angular = -0.1 * 180 / M_PI;
                                                }
                                                vel_pub.publish(vel_msg);
                                        }
                                        else
                                        {
                                                geometry_msgs::Twist vel_msg;
                                                if(target_yaw - robot_pose.yaw > 0)
                                                {
                                                        vel_msg.angular.z = 0.5;
                                                }
                                                else
                                                {
                                                        vel_msg.angular.z = -0.5;
                                                }
                                                vel_msg.linear.x = 0.0;
                                                vel_pub.publish(vel_msg);
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
                                                vel_pub.publish(vel_msg);
                                        }
                                        else
                                        {
                                                geometry_msgs::Twist vel_msg;
                                                vel_msg.angular.z = 0;
                                                vel_msg.linear.x = 0.0;
                                                vel_pub.publish(vel_msg);
                                        }
                                        ROS_INFO("goto goal success");
                                        goto_complete_flag = true;
                                        Run_State = Common_Control::RunStateValue::Tracking;
                                }
                        }
                        // rate.sleep();
                }

                void Tracking_Path()
                {
                        current_point.first.first = robot_pose.x;
                        current_point.first.second = robot_pose.y;
                        current_point.second = robot_pose.yaw;

                        current_index = GetMinDistance(current_point, path_vector);
                        goal_point = path_vector[current_index];
                        ROS_INFO("Goal_Point: (X=%f,Y= %f,YAW= %f)", goal_point.first.first, goal_point.first.second, goal_point.second);
                        ROS_INFO("Robot_Pose: (X=%f,Y= %f,YAW= %f)", robot_pose.x, robot_pose.y, robot_pose.yaw);
                        if (current_index == path_vector.size() - 1)
                        {
                                ROS_INFO("Reached the last point. Stopping the robot.");
                                goto_complete_flag = false;
                                Run_State = Common_Control::RunStateValue::Waitting;
                        }

                        double e_y = Get_Distance(current_point, goal_point);
                        double delta = 0.0;
                        double kp = 1;
                        double truck_speed = 0.1;
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
                                double delta_e = atan2(1.0 * e_y, 0.1);
                                delta = delta_e + 1*theta_e;
                        }

                        if (delta > M_PI)
                        {
                                delta -= 2 * M_PI;
                        }
                        else if (delta < -M_PI)
                        {
                                delta += 2 * M_PI;
                        }

                        ROS_INFO("delta: %f", delta);

                        if (robot_type)
                        {
                                yhs_can_msgs::ctrl_cmd vel_msg;
                                vel_msg.ctrl_cmd_gear = 06;
                                vel_msg.ctrl_cmd_x_linear = truck_speed;
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
                        vel_msg.linear.x = 0.1;
                        vel_msg.angular.z = delta;
                        if (odom_callback_flag)
                        {
                                vel_pub.publish(vel_msg);
                        }
                        }
                }
                void Goto_Watting_Point(path_type point)
                {
                        path_type stop_goal = point;
                        double target_x = stop_goal.first.first;
                        double target_y = stop_goal.first.second;
                        double target_yaw = stop_goal.second;

                        ROS_INFO("Goto_Goal: %f, %f, %f", target_x, target_y, target_yaw);
                        ROS_INFO("Robot_Pose: %f, %f, %f", robot_pose.x, robot_pose.y, robot_pose.yaw);

                        double distance = sqrt(pow(target_x - robot_pose.x, 2) + pow(target_y - robot_pose.y, 2));
                        ROS_INFO("distance:%f", distance);

                        if (distance >= 0.05)
                        {
                                double angle_to_goal = atan2(target_y - robot_pose.y, target_x - robot_pose.x);
                                geometry_msgs::Twist vel_msg;
                                vel_msg.angular.z = 4 * (angle_to_goal - robot_pose.yaw);
                                vel_msg.linear.x = 0.1;
                                vel_pub.publish(vel_msg);
                        }
                        else
                        {
                                ROS_INFO("yaw_err: %f", std::abs(target_yaw - robot_pose.yaw));
                                if (std::abs(target_yaw - robot_pose.yaw) > 0.05)
                                {
                                        geometry_msgs::Twist vel_msg;
                                        if (target_yaw - robot_pose.yaw > 0)
                                        {
                                                vel_msg.angular.z = 0.5;
                                        }
                                        else
                                        {
                                                vel_msg.angular.z = -0.5;
                                        }
                                        vel_msg.linear.x = 0.0;
                                        vel_pub.publish(vel_msg);
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
                                                vel_pub.publish(vel_msg);
                                        }
                                        else
                                        {
                                                geometry_msgs::Twist vel_msg;
                                                vel_msg.angular.z = 0;
                                                vel_msg.linear.x = 0.0;
                                                vel_pub.publish(vel_msg);
                                        }
                                        ROS_INFO("goto point success");
                                        goto_complete_flag = true;
                                        Run_State = Common_Control::RunStateValue::Stop;
                                }
                        }
                }
        };
}

int main(int argc, char *argv[])
{
        ros::init(argc, argv, "robot1_control");

        std::string robot_name = "robot1";
        std::string odom_topic = "/robot1/odom";
        std::string robot_topic = "/robot1/cmd_vel";
        // std::string odom_topic = "/four_wheel_steering_controller/odom";
        // std::string robot_topic = "/four_wheel_steering_controller/cmd_vel";
        // std::string odom_topic = "/current_pose";
        // std::string robot_topic = "/ctrl_cmd";
        std::string pose_pub_topic = "robot1_pose";

        Robot1_Control::Robot1Control robot1(robot_name, odom_topic, robot_topic, pose_pub_topic);
        robot1.Init(); // 初始化
        robot1.Run(); // 运行

        return 0;
}