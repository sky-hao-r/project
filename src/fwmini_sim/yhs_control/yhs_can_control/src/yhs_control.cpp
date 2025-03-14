#include "ros/ros.h"
#include"yhs_can_msgs/ctrl_cmd.h"
#include "geometry_msgs/Twist.h"
#include"chrono"
#include"thread"

int main(int argc, char *argv[])
{
        ros::init(argc, argv, "fwmini");
        ros::NodeHandle nh;
        ros::Publisher pub = nh.advertise<yhs_can_msgs::ctrl_cmd>("/ctrl_cmd",10);
        // ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/four_wheel_steering_controller/cmd_vel", 10);
        ros::Rate rate(100);
        while (ros::ok())
        {
                for (int i = 0; i < 200; ++i)
                {

                }
                for (int i = 0; i < 1000;++i)
                {
                        yhs_can_msgs::ctrl_cmd msg;
                        msg.ctrl_cmd_gear = 06;
                        msg.ctrl_cmd_x_linear = -0.03;
                        msg.ctrl_cmd_y_linear = 0;
                        msg.ctrl_cmd_z_angular = 0.0;

                        pub.publish(msg);
                        rate.sleep();
                }

                ros::spinOnce();
        }

        return 0;
}
