#include <ros/ros.h>
#include <std_msgs/Time.h>
#include <can_msgs/Frame.h>
#include <mutex>
#include <thread>
#include <chrono>
#include <cstring>
#include <sys/socket.h>  // 包含套接字相关功能
#include <linux/can.h>   // 包含CAN协议族相关定义
#include <net/if.h>      // 包含接口相关定义
#include <unistd.h>      // 包含一些系统调用
#include <sys/ioctl.h>     // 确保包含此头文件
#include <fcntl.h>  // 包含fcntl.h头文件，提供了文件控制功能
#include <errno.h>  // 包含errno变量
#include <signal.h>
#include <atomic>
#include "odom1/underpan_speed.h"
#include "odom1/underpan_code_w.h"
// 二车最大速度 0.044 m/s
/*
can 调试：
    can-untils
虚拟can：
    sudo modprobe vcan //加载虚拟can模块
    sudo ip link add dev vcan0 type vcan //增加虚拟can设备
    sudo ip link set up vcan0 //启动虚拟can设备
    
    sudo ip link set down vcan0 //关闭虚拟can设备
    sudo ip link delete vcan0 type can //删除虚拟can设备
    sudo modprobe -r vcan //不用执行（删除模块）
实体can：
    sudo ip link set can0 type can bitrate 500000 //设置can0波特率
    sudo ip link set can0 up //启动can0设备
监测发送：
    candump vcan0 //监测命令
    cansend vcan0 310#00010001 // 发送命令
    candump 默认显示 标准 CAN ID 的低 16 位，对于 扩展 CAN ID，它只会显示低 16 位部分，
            因此看到的 00001310 在代码中是 80001310
*/

int can_socket;
ros::Publisher underpan_code_w_pub;
std::mutex can_mutex;

void signal_handler(int signum)
{
    ROS_WARN("underpan-Interrupt signal (%d) received. Exiting...", signum);
    ros::shutdown();
}

// 数据缓存结构体
struct UnderpanDataCache {
    std_msgs::Time undepan_time; 
    double left_speed, right_speed;
    double left_code_w, right_code_w;
    
    bool has_left_speed = false;
    bool has_right_speed = false;
    bool has_left_code_w = false;
    bool has_right_code_w = false;
    
    void reset() {
        has_left_speed = has_right_speed 
        = has_left_code_w = has_right_code_w = false;
    }
};

UnderpanDataCache data_cache;

int16_t extractS16(const uint8_t* data, int start_bit) {
    // 计算字节起始位置（设备使用大端序）
    const int byte_index = start_bit / 8;
    // 组合高/低字节并转换类型
    return (int16_t)(
        (data[byte_index] << 8) |  // 高字节左移
        data[byte_index + 1]       // 低字节直接拼接
    );
}

void encodeInt16(int16_t decimal, uint8_t* output) {
    // 大端序编码（Modbus标准）
    output[0] = (decimal >> 8) & 0xFF; // 高字节
    output[1] = decimal & 0xFF;        // 低字节
}

// 初始化 CAN 套接字
int initCANSocket(const char* can_interface) {
    int can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket < 0) {
        perror("underpan-SocketCAN socket creation failed");
        std::cout<<"-----------------"<<std::endl;
        return -1;
    }
    struct ifreq ifr;
    strncpy(ifr.ifr_name, can_interface, IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0'; // 确保字符串终止
    if (ioctl(can_socket, SIOCGIFINDEX, &ifr) < 0) {
        ROS_ERROR("underpan-CAN interface ioctl failed: %s", strerror(errno));
        std::cout<<"-----------------"<<std::endl;
        close(can_socket);
        return -1;
    }
    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(can_socket, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("underpan-SocketCAN bind failed");
        std::cout<<"-----------------"<<std::endl;
        close(can_socket);
        return -1;
    }
    return can_socket;
}
// 设置套接字为非阻塞模式
int setSocketNonBlocking(int can_socket) { 
    int flags = fcntl(can_socket, F_GETFL, 0);  // 获取当前套接字的标志
    if (flags == -1) {
        perror("underpan-fcntl F_GETFL failed");
        return -1;
    }
    
    flags |= O_NONBLOCK;  // 设置非阻塞标志
    if (fcntl(can_socket, F_SETFL, flags) == -1) {
        perror("underpan-fcntl F_SETFL failed");
        return -1;
    }
    return 0;
}
// 解析履带速度数据
void speedData(const can_msgs::Frame& frame) {
    data_cache.left_speed = extractS16(frame.data.data(), 0) / 1000;
    data_cache.right_speed = extractS16(frame.data.data(), 16) / 1000;
    data_cache.has_left_speed = data_cache.has_right_speed = true;
}

// 解析编码器数据
void codewData(const can_msgs::Frame& frame) {
    data_cache.left_code_w = static_cast<double>(extractS16(frame.data.data(), 0)) * 60.0 / 1024.0 * 10.0*0.0002;
    data_cache.right_code_w = static_cast<double>(extractS16(frame.data.data(), 16)) * 60.0 / 1024.0 * 10.0*0.0002;
    // data_cache.left_code_w = static_cast<double>(extractS16(frame.data.data(), 0)) * 0.0001875;
    // data_cache.right_code_w = static_cast<double>(extractS16(frame.data.data(), 32)) * 0.0001875;
    data_cache.has_left_code_w = true;
    data_cache.has_right_code_w = true;
}

// 解析 CAN 数据帧
void parseCANFrame(const can_msgs::Frame& frame, ros::Publisher& pub) {
    switch (frame.id) {
        // case 0x80001312://发送给履带的实际速度
        //     speedData (frame);
        //     break;
        case 0x80001310://编码器返回的转速
            codewData (frame);
            break;
        // default:
        //     ROS_WARN("underpan-Unhandled CAN ID: 0x%03X", frame.id);
    }
    if (data_cache.has_left_code_w  && data_cache.has_right_code_w) {
        std::cout << "underpan-data_cache.left_code_w   : " << data_cache.left_code_w << std::endl;
        std::cout << "underpan-data_cache.right_code_w  : " << data_cache.right_code_w << std::endl;
        odom1::underpan_code_w code_w_msg;
        code_w_msg.header.stamp = ros::Time::now();
        code_w_msg.left_code_w = data_cache.left_code_w;
        code_w_msg.right_code_w = data_cache.right_code_w;
        pub.publish(code_w_msg);
        if (data_cache.has_left_speed && data_cache.has_right_speed) {
            std::cout << "underpan-data_cache.left_speed   : " << data_cache.left_speed << std::endl;
            std::cout << "underpan-data_cache.right_speed  : " << data_cache.right_speed << std::endl;
            data_cache.reset();
        }
        data_cache.reset();
    }
}
// 速度期望回调函数
void underpan_speed_Callback(const odom1::underpan_speed::ConstPtr& msg) {
    ROS_INFO("underpan-Received underpan speed message");
    struct can_frame frame;
    frame.can_id = 0x80001312;
    frame.can_dlc = 4;
    uint8_t left_encoded[2];
    uint8_t right_encoded[2];
    int16_t left_speed = static_cast<int16_t>(msg->left_speed * 1000);
    int16_t right_speed = static_cast<int16_t>(msg->right_speed * 1000);
    encodeInt16(left_speed, left_encoded);
    encodeInt16(right_speed, right_encoded);
    // 将编码后的数据放入 CAN 帧
    frame.data[0] = left_encoded[0];
    frame.data[1] = left_encoded[1];
    frame.data[2] = right_encoded[0];
    frame.data[3] = right_encoded[1];
    frame.can_id |= CAN_EFF_FLAG;  // 使用扩展帧
    // 调用发送函数，将 CAN 帧写入总线
    int nbytes = write(can_socket, &frame, sizeof(struct can_frame));
    if(nbytes != sizeof(struct can_frame)) {
        perror("underpan-Write error");
    } else {
        ROS_INFO("underpan-CAN frame sent successfully");
    }
}
void read_can_underpan(const ros::TimerEvent& event) {
    struct can_frame frame;
    // ROS_INFO("underpan-Reading CAN data...");
    int nbytes = read(can_socket, &frame, sizeof(frame));// 如果是阻塞模式，则会阻塞整个线程
    if (nbytes > 0) {
        can_msgs::Frame ros_frame;
        ros_frame.header.stamp = ros::Time::now();
        ros_frame.id = frame.can_id;
        ros_frame.dlc = frame.can_dlc;
        memcpy(ros_frame.data.data(), frame.data, frame.can_dlc);

        {
            std::lock_guard<std::mutex> lock(can_mutex);
            parseCANFrame(ros_frame, underpan_code_w_pub);
        }
    }else if (nbytes == -1 && errno == EAGAIN) {
        // 没有数据，继续等待
        // ROS_INFO("No CAN data available.");
    } else if (nbytes < 0) {
        // 发生了其他错误
        ROS_ERROR("underpan-Failed to read CAN frame: %s", strerror(errno));
    }
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "underpan");
    ros::NodeHandle nh;
    // signal(SIGINT, signal_handler);

    std::string can_interface;
    nh.param<std::string>("can_interface", can_interface, "can1");

    can_socket = initCANSocket(can_interface.c_str());
    if (can_socket < 0) return -1;
    // 设置非阻塞模式
    if (setSocketNonBlocking(can_socket) < 0) {
        ROS_ERROR("underpan-Failed to set CAN socket to non-blocking mode");
        return -1;
    }

    ROS_INFO("underpan-CAN socket initialized");

    underpan_code_w_pub = nh.advertise<odom1::underpan_code_w>("underpan_code_w", 1);

    // ROS_INFO("Starting to subscribe to /underpan_speed...");
    ros::Subscriber speed_sub = nh.subscribe<odom1::underpan_speed>(
        "/underpan_speed",1,underpan_speed_Callback);
    ros::Timer timer_actuator = nh.createTimer(ros::Duration(0.001), read_can_underpan);
    // ros::Timer timer_actuator = nh.createTimer(ros::Duration(0.05), boost::bind(read_can_underpan, _1, underpan_code_w_pub));// 会阻塞
    ros::spin();

    close(can_socket);

    return 0;
}
