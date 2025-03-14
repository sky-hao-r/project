#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include"chrono"
#include"thread"

int main(int argc, char *argv[])
{
        ros::init(argc, argv, "fwmini");
        ros::NodeHandle nh;
        ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/four_wheel_steering_controller/cmd_vel", 10);
        ros::Rate rate(100);
        while (ros::ok())
        {
                for (int i = 0; i < 200; ++i)
                {

                }
                for (int i = 0; i < 800;++i)
                {
                        geometry_msgs::Twist msg;
                        msg.linear.x = 0.3;
                        msg.linear.y = 0;
                        msg.angular.z = 0.0;

                        pub.publish(msg);
                        rate.sleep();
                }

                for (int i = 0; i < 800; ++i)
                {
                        geometry_msgs::Twist msg;
                        msg.linear.x =-0.3;
                        msg.linear.y = 0;
                        msg.angular.z = 0.0;

                        pub.publish(msg);
                        rate.sleep();
                }

                for (int i = 0; i < 800; ++i)
                {
                        geometry_msgs::Twist msg;
                        msg.linear.x = 0;
                        msg.linear.y= 0;
                        msg.angular.z = 1.0;
                        pub.publish(msg);
                        rate.sleep();
                }
                for (int i = 0; i < 800; ++i)
                {
                        geometry_msgs::Twist msg;
                        msg.linear.x = 0;
                        msg.linear.y = 0;
                        msg.angular.z = -1.0;

                        pub.publish(msg);
                        rate.sleep();
                }

                for (int i = 0; i < 800; ++i)
                {
                        geometry_msgs::Twist msg;
                        msg.linear.x = 0.3;
                        msg.linear.y = 0;
                        msg.angular.z = -1.0;

                        pub.publish(msg);
                        rate.sleep();
                }

                for (int i = 0; i < 800; ++i)
                {
                        geometry_msgs::Twist msg;
                        msg.linear.x = 0.3;
                        msg.linear.y = 0;
                        msg.angular.z = 1.0;

                        pub.publish(msg);
                        rate.sleep();
                }

                // for (int i = 0; i < 500; ++i)
                // {
                //         geometry_msgs::Twist msg;
                //         msg.linear.x = 0.3;
                //         msg.linear.y = 0.3;
                //         msg.angular.z = 0.0;

                //         pub.publish(msg);
                //         rate.sleep();
                // }
                // for (int i = 0; i < 500; ++i)
                // {
                //         geometry_msgs::Twist msg;
                //         msg.linear.x = 0.3;
                //         msg.linear.y = -0.3;
                //         msg.angular.z = 0.0;

                //         pub.publish(msg);
                //         rate.sleep();
                // }
                ros::spinOnce();
        }

        return 0;
}
