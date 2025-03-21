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

// 轨迹点
struct TrajectoryPoint {
    double x, y;
};

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
    json underpan_speed_json(double v_left, double v_right) {
        return {
            {"v_left", v_left},
            {"v_right", v_right}
        };
    }
    json underpan_position_json(double x, double y,double yaw) {
        return {
            {"x", x},
            {"y", y},
            {"yaw", yaw}
        };
    }
    // ROS 回调中调用，用于更新 ROS 话题数据
    void plc_running_ROSData(const std_msgs::Float64::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(ros_mutex);
        head_plc_running = msg->data;
    }
    void speed_ROSData(const odom1::underpan_code_w::ConstPtr& data) {
        std::lock_guard<std::mutex> lock(ros_mutex);
        underpan_code_w.left_code_w = data->left_code_w;
        underpan_code_w.right_code_w = data->right_code_w;
    }
    void position_ROSData(const geometry_msgs::PoseStamped::ConstPtr& data) {
        std::lock_guard<std::mutex> lock(ros_mutex);
        underpan_position.pose.position.x = data->pose.position.x;
        underpan_position.pose.position.y = data->pose.position.y;
        underpan_position.pose.position.z = data->pose.position.z;
        underpan_position.pose.orientation.z = data->pose.orientation.z;
    }
    void carState_ROSData(const std_msgs::String::ConstPtr& data){
        std::lock_guard<std::mutex> lock(ros_mutex);
        car_state = data->data;
    }

private:
    websocket_client client;
    string server_uri;
    //轨迹点数组，存放轨迹点
    vector<TrajectoryPoint> trajectory;
    //限制轨迹点最大数量
    const size_t maxTrajectoryPoints = 50;

    // 用于存储 ROS 接收到的数据
    std::string head_plc_running;
    std::mutex ros_mutex;
    odom1::underpan_code_w underpan_code_w;
    geometry_msgs::PoseStamped underpan_position;
    std::string car_state;

    // 连接成功回调
    void on_open(connection_hdl hdl) {
        cout << "成功连接到服务器" << endl;
        this->connection = hdl;
        start_sending();
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

    // 发送车辆、电机状态和轨迹数据
    void start_sending() {
        thread([this]() {
            srand(time(nullptr));
            while (true) {
                json message;
                // 从 ROS 回调中获取数据
                {
                    std::lock_guard<std::mutex> lock(ros_mutex);
                    message["type"] = "status_update";
                    message["head_plc_running"] = head_plc_running;
                    message["head_speed"] = underpan_speed_json(underpan_code_w.left_code_w, underpan_code_w.right_code_w);
                    message["Head_Trajectory"] = underpan_position_json(underpan_position.pose.position.x, 
                                                                    underpan_position.pose.position.y,
                                                                    underpan_position.pose.orientation.z);
                    message["Head_state"] = car_state;
                }
                client.send(connection, message.dump(), frame::opcode::text);
                cout << "已发送状态信息和轨迹数据: " << message.dump() << endl;
                //每两秒发送一次
                // this_thread::sleep_for(chrono::seconds(2));
                this_thread::sleep_for(chrono::milliseconds(10));

            }
        }).detach();
    }

    connection_hdl connection;
};

int main(int argc, char** argv) {
    // 初始化 ROS 节点
    ros::init(argc, argv, "head_websocket_ros_client");
    ros::NodeHandle nh;

    string server_address = "ws://101.200.15.67:3000";
    cout << "尝试连接到: " << server_address << endl;
    WebSocketClient client(server_address);
    client.connect();
    // 表示 头车 plc 工作状态
    ros::Subscriber plc_sub = nh.subscribe<std_msgs::Float64>("kongyaji", 1,
        [&client](const std_msgs::Float64::ConstPtr& msg) {
            client.plc_running_ROSData(msg);
            cout << "收到 head plc running ROS 数据: " << msg->data << endl;
        }
    );
    // 发送底盘速度
    ros::Subscriber underpan_speed_sub = nh.subscribe<odom1::underpan_code_w>("underpan_code_w", 1,
        [&client](const odom1::underpan_code_w::ConstPtr& msg) {
            client.speed_ROSData(msg);
            cout << "收到 head speed ROS 数据: " << msg->left_code_w<<", "
                                                <<msg->right_code_w << endl;
        }
    );
    // 发送底盘轨迹
    ros::Subscriber underpan_position_sub = nh.subscribe<geometry_msgs::PoseStamped>("current_pose", 1,
        [&client](const geometry_msgs::PoseStamped::ConstPtr& msg) {
            client.position_ROSData(msg);
            cout << "收到 head speed ROS 数据: " << msg->pose.position.x << ", "
                                                << msg->pose.position.y << ", "
                                                << msg->pose.position.z << ", "
                                                << msg->pose.orientation.z << endl;
        }
    );
    ros::Subscriber underpan_state_sub = nh.subscribe<std_msgs::String>("car1_state", 1,
        [&client](const std_msgs::String::ConstPtr& msg) {
            client.carState_ROSData(msg);
            cout << "收到 head speed ROS 数据: " << msg->data << endl;
        }
    );

    ros::spin();

    return 0;
}
