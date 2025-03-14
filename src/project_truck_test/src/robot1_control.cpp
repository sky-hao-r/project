#include "common_control.h" // 引用公共代码库
#include "differ_robot/PoseRPY.h"

namespace Robot1_Control
{
        class Robot1Control : public Common_Control::CommonControl
        {
        public:
                Robot1Control(const std::string &robot_name, const std::string &odom_topic, const std::string &robot_topic, const std::string &pose_pub_topic)
                    : CommonControl(robot_name, odom_topic, robot_topic, pose_pub_topic)
                {
                        // path_vector = ReadFile("/home/hmh/project_truck/src/project_truck/Data/output1.txt");
                        // path_vector = ReadFile("/home/hmh/project_truck/src/project_truck/Data/input_1.csv");
                        path_vector = ReadFile("/home/hmh/project_truck/src/project_truck/Data/output_xin.txt");
                        watting_point={{34.0,10.0},M_PI/2};
                        watting_point_true={{34.0,10.0},M_PI/2};
                }
                void Init()
                {
                        CommonControl::Init();
                        ros::NodeHandle nh;

                        state_pub = nh.advertise<std_msgs::String>("robot1_state", 1);
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

                                std_msgs::String state_msg;
                                if (Robot_State == Common_Control::StateValue::Idle)
                                {
                                        state_msg.data = "Idle";
                                }
                                else if (Robot_State == Common_Control::StateValue::Run)
                                {
                                        state_msg.data = "Run";
                                }
                                else if (Robot_State == Common_Control::StateValue::Pause)
                                {
                                        state_msg.data = "Pause";
                                }
                                state_pub.publish(state_msg);

                                switch (Robot_State)
                                {
                                case Common_Control::StateValue::Idle:
                                        ROS_INFO("Robot_Pose: %f, %f, %f", robot_pose.x, robot_pose.y, robot_pose.yaw);
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
                ros::Publisher state_pub;
                path_type watting_point;
                path_type watting_point_true;
                std_msgs::String state_msg;

                void Running_State()
                {
                        switch (Run_State)
                        {
                                case Common_Control::RunStateValue::Goto:
                                        ROS_INFO("Robot1 is going to the first goal point.");
                                        /*
                                                Goto_Goal 第一个参数为终点坐标，第二个参数为距离目标点最小距离，第三个参数为速度，第四个参数为角速度调整参数,第五个参数判断是不是goto状态
                                        */
                                        if (robot_type)
                                        {
                                                // Goto_Goal(path_vector[0], 0.05, 0.1, 1.0,1);
                                                Goto_Goal(path_vector[0], 0.1, 0.1, 1, 1);
                                        }
                                        else
                                        {
                                                Goto_Goal(path_vector[0], 0.05, 0.3, 4.0, 1);
                                        }
                                        break;

                                case Common_Control::RunStateValue::Tracking:
                                        ROS_INFO("Robot1 is tracking the path.");
                                        /*
                                                Tracking_Path_stanley 第一个参数为路径点，第二个参数为速度，第三个参数为角速度调整参数
                                                Tracking_Path_pure_pursuit 第一个参数为路径点，第二个参数为前瞻距离，第三个参数为速度
                                        */
                                        if (robot_type)
                                        {
                                                // Tracking_Path_stanley(path_vector, 0.1, 1.0,1.0);
                                                Tracking_Path_pure_pursuit(path_vector, 1, 0.1, 0.0);
                                        }
                                        else
                                        {
                                                // Tracking_Path_stanley(path_vector, 0.1,1.0,1.0);
                                                Tracking_Path_pure_pursuit(path_vector, 0.05, 0.1, 1.0);
                                        }
                                
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

                void Goto_Watting_Point(path_type point)
                {
                        if (robot_type)
                        {
                                Goto_Goal(point, 0.05, 0.1, 1.0,0);
                        }
                        else
                        {
                                Goto_Goal(point, 0.05, 0.1, 1.0,0);
                        }
                        
                }
        };
}

int main(int argc, char *argv[])
{
        ros::init(argc, argv, "robot1_control");

        std::string robot_name = "robot1";
        // std::string odom_topic = "/robot1/current_pose";
        // std::string robot_topic = "/robot1/cmd_vel";
        // std::string odom_topic = "/four_wheel_steering_controller/odom";
        // std::string robot_topic = "/four_wheel_steering_controller/cmd_vel";
        std::string odom_topic = "/current_pose";
        std::string robot_topic = "/ctrl_cmd";
        std::string pose_pub_topic = "robot1_pose";

        Robot1_Control::Robot1Control robot1(robot_name, odom_topic, robot_topic, pose_pub_topic);
        robot1.Init(); // 初始化
        robot1.Run(); // 运行

        return 0;
}