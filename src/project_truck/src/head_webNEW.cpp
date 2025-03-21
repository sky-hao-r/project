#include <iostream>
#include <websocketpp/config/asio_client.hpp>
#include <websocketpp/client.hpp>
#include <nlohmann/json.hpp>
#include <vector>
#include <cstdlib>
#include <ctime>

// ROS 相关头文件
#include <ros/ros.h>
#include <std_msgs/Float64.h>
// 底盘速度相关
#include "odom1/underpan_code_w.h"
// 定位相关
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

using namespace websocketpp;
using namespace std;
using json = nlohmann::json;

typedef client<config::asio_client> websocket_client;
typedef websocketpp::connection_hdl connection_hdl;

// json underpan_speed_json(double v_left, double v_right) {
//     return {
//         {"v_left", v_left},
//         {"v_right", v_right}
//     };
// }
json underpan_position_json(double x, double y,double yaw) {
    return {
        {"x", x},
        {"y", y},
        {"yaw", yaw}
    };
}
// WebSocket 客户端
class WebSocketClient {
public:
    //构造函数，传入服务器地址
    WebSocketClient(const string& uri) : server_uri(uri) {
        client.init_asio();
        client.set_open_handler(bind(&WebSocketClient::on_open, this, placeholders::_1));
        client.set_message_handler(bind(&WebSocketClient::on_message, this, placeholders::_1, placeholders::_2));
        client.set_close_handler(bind(&WebSocketClient::on_close, this, placeholders::_1));
        client.set_fail_handler(bind(&WebSocketClient::on_fail, this, placeholders::_1));
    }
    //连接服务器函数
    void connect() {
        websocketpp::lib::error_code ec;
        auto con = client.get_connection(server_uri, ec);
        if (ec) {
            cerr << "连接错误: " << ec.message() << endl;
            return;
        }
        client.connect(con);
        thread([this] { client.run(); }).detach();
    }
    void send(const json& message) {
        try {
            client.send(connection, message.dump(), frame::opcode::text);
            cout << "已发送数据: " << message.dump() << endl;
        } catch (std::exception & e) {
            cerr << "发送数据失败: " << e.what() << endl;
        }
    }
    private:
    websocket_client client;
    string server_uri;
    connection_hdl connection;
    // 连接成功回调
    void on_open(connection_hdl hdl) {
        cout << "成功连接到服务器" << endl;
        this->connection = hdl;
    }
    // 接收消息回调
    void on_message(connection_hdl, websocket_client::message_ptr msg) {
        cout << "收到服务器消息: " << msg->get_payload() << endl;
    }
    // 断开连接回调
    void on_close(connection_hdl) {
        cout << "连接已关闭" << endl;
    }
    // 连接失败回调
    void on_fail(connection_hdl) {
        cout << "WebSocket 连接失败" << endl;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "head_websocket_ros_client");
    ros::NodeHandle nh;

    string server_address = "ws://101.200.15.67:3000";
    cout << "尝试连接到: " << server_address << endl;
    WebSocketClient client(server_address);
    client.connect();

    // 表示 头车 plc 工作状态
    // ros::Subscriber plc_sub = nh.subscribe<std_msgs::Float64>("kongyaji", 1,
    //     [&client](const std_msgs::Float64::ConstPtr& msg) {
    //         cout << "收到 head plc running ROS 数据: " << msg->data << endl;
    //         std::string head_plc_running;
    //         json message;
    //         message["type"] = "status_update";
    //         message["head_plc_running"] = head_plc_running;
    //         client.send(message);
    //         cout << "已发送状态信息和轨迹数据: " << message.dump() << endl;
    //     }
    // );
    // 发送底盘定位
    ros::Subscriber underpan_position_sub = nh.subscribe<geometry_msgs::PoseStamped>("current_pose", 1,
        [&client](const geometry_msgs::PoseStamped::ConstPtr& msg) {
            cout << "收到 head speed ROS 数据: " << msg->pose.position.x << ", "
                                                << msg->pose.position.y << ", "
                                                << msg->pose.position.z << ", "
                                                << msg->pose.orientation.z << endl;
            geometry_msgs::PoseStamped underpan_position;
            underpan_position.pose.position.x = msg->pose.position.x;
            underpan_position.pose.position.y = msg->pose.position.y;
            underpan_position.pose.position.z = msg->pose.position.z;
            underpan_position.pose.orientation.z = msg->pose.orientation.z;
            json message;
            message["type"] = "status_update";
            message["Head_Trajectory"] = underpan_position_json(underpan_position.pose.position.x, 
                                                            underpan_position.pose.position.y,
                                                            underpan_position.pose.orientation.z);
            client.send(message);
            cout << "已发送状态信息和轨迹数据: " << message.dump() << endl;
        }
    );
    ros::Subscriber underpan_state_sub = nh.subscribe<std_msgs::String>("car1_state", 1,
        [&client](const std_msgs::String::ConstPtr& msg) {
            std::string car_state;
            car_state = msg->data;
            cout << "收到 head car_state ROS 数据: " << msg->data << endl;
            json message;
            message["type"] = "status_update";
            message["Head_state"] = car_state;
            client.send(message);
            cout << "已发送底盘状态数据: " << message.dump() << endl;
        }
    );

    ros::spin();

    return 0;
}
