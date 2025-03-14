#include"ros/ros.h"
#include"geometry_msgs/Twist.h"
#include"odom1/underpan_speed.h"
#include"odom1/underpan_code_w.h"

double fl = 0.0, fr = 0.0, B = 2.9; //fl和fr分别为左右轮滑移参数,B为轮距

void callback(const odom1::underpan_code_w::ConstPtr& msg)
{
        ROS_INFO("左轮速度：%f,右轮速度：%f", msg->left_code_w, msg->right_code_w);
}

int main(int argc, char *argv[])
{
        setlocale(LC_ALL,"");
        ros::init(argc, argv, "car_test");
        ros::NodeHandle nh;
        ros::Publisher pub = nh.advertise<odom1::underpan_speed>("underpan_speed", 1000);
        ros::Subscriber sub = nh.subscribe("underpan_code_w", 1000, callback);
        ros::Rate rate(10);
        while(ros::ok())
        {
                geometry_msgs::Twist speed;
                double vx = speed.linear.x = -0.01;
                double vw = speed.angular.z = 0.001;
                odom1::underpan_speed msg;
                msg.left_speed = (2*vx-B*vw)*(1-fl)/2;
                msg.right_speed = (2*vx+B*vw)/(2*(1-fr));
                ROS_INFO("左轮速度：%f,右轮速度：%f", msg.left_speed, msg.right_speed);
                pub.publish(msg);
                rate.sleep();
                ros::spinOnce();
        }
        return 0;
}
