#include <iostream>
#include <websocketpp/config/asio_client.hpp>
#include <websocketpp/client.hpp>
#include <nlohmann/json.hpp>

#include "project_truck/RobotPose.h"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
using namespace websocketpp;
using namespace std;
using json = nlohmann::json;

typedef client<config::asio_client> websocket_client;
typedef websocketpp::connection_hdl connection_hdl;

class WebSocketReceiver {
public:
    WebSocketReceiver(const string& uri, ros::NodeHandle &nh) : server_uri(uri) {
        client.init_asio();
        client.set_message_handler(bind(&WebSocketReceiver::on_message, this, placeholders::_1, placeholders::_2));
        client.set_open_handler(bind(&WebSocketReceiver::on_open, this, placeholders::_1));
        client.set_close_handler(bind(&WebSocketReceiver::on_close, this, placeholders::_1));
        client.set_fail_handler(bind(&WebSocketReceiver::on_fail, this, placeholders::_1));
        pose_pub_ = nh.advertise<project_truck::RobotPose>("car1_pose", 1);
        carState_pub_ = nh.advertise<std_msgs::String>("car1_state", 1);
    }

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

private:
    websocket_client client;
    string server_uri;
    ros::Publisher pose_pub_;
    ros::Publisher carState_pub_;

    void on_open(connection_hdl) {
        cout << "成功连接到服务器: " << server_uri << endl;
    }

    void on_message(connection_hdl, websocket_client::message_ptr msg) {
        try {
            json received_data = json::parse(msg->get_payload());
            // if (received_data.contains("Head_Trajectory")) {
            //     cout << "收到轨迹数据: " << endl;
            //     for (const auto& point : received_data["Head_Trajectory"]) {
            //         cout << "x: " << point["x"] << ", y: " << point["y"] <<", yaw:"<<point["yaw"]<< endl;
            //     }
            // }
            if (received_data.contains("Head_Trajectory")) {
                json pos = received_data["Head_Trajectory"];
                double x = pos["x"];
                double y = pos["y"];
                // 此处将 yaw 当作 z 坐标发布，你可以根据实际情况调整
                double z = pos["yaw"];
                cout << "收到头位置信息:" << endl;
                cout << "x: " << x << ", y: " << y << ", z: " << z << endl;
                project_truck::RobotPose pose_msg;
                pose_msg.x = x;
                pose_msg.y = y;
                pose_msg.yaw = z;
                pose_pub_.publish(pose_msg);
            } else {
                cout << "消息中不包含 Head_Trajectory 字段" << endl;
            }
            if (received_data.contains("Head_state")) {
                json pos = received_data["Head_state"];
                std::string car_state = pos;
                cout << "收到状态消息: " << car_state << endl;
                std_msgs::String carState_msg;
                carState_msg.data = car_state;
                carState_pub_.publish(carState_msg);
            } else {
                cout << "消息中不包含 Head_Trajectory 字段" << endl;
            }
        } catch (std::exception& e) {
            cerr << "解析 WebSocket 消息出错: " << e.what() << endl;
        }
    }

    void on_close(connection_hdl) {
        cout << "连接已关闭" << endl;
    }

    void on_fail(connection_hdl) {
        cout << "WebSocket 连接失败" << endl;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "websocket_receiver");
    ros::NodeHandle nh;
    string server_address = "ws://101.200.15.67:3000";
    WebSocketReceiver client(server_address,nh);
    client.connect();

    ros::spin();
    return 0;
}
