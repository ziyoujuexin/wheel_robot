#include <iostream>
#include <serial/serial.h> // 使用 serial 库来处理串口通信
#include <string>
#include <vector>
#include <unistd.h>
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "rclcpp/rclcpp.hpp"
#define HAPPY 4
#define CONFUSE 5
#define SAD 6
constexpr uint16_t FRAME_HEADER = 0x55AA;
constexpr uint8_t FRAME_TAIL = 0xBB;
constexpr uint8_t SENDER_ID = 0x01;
constexpr uint8_t DATA_TYPE_MOTION = 0x02;
constexpr size_t FRAME_LENGTH = 7;

class ScreenNode : public rclcpp::Node
{
public:
    ScreenNode():Node("screenNode"){}
    ScreenNode(const char* nodename);
    void play_animation(const std::string& anim_name);
    ~ScreenNode(){ser.close();}
    std::vector<std::string> animation = 
    {
        "[L]开心_8_36s_x1.1",
        "[L]张望1_4_54s_x1.28",
        "[L]伤心_5_59s_x1.45"
    };
private:
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr modelSub;
    serial::Serial ser;
    std::string serial_port = "/dev/ttyCH341USB1";
    int baudrate = 115200;
    void send_frame(uint8_t data_type, const std::vector<uint8_t>& payload);
    void modelSubCallback(const std_msgs::msg::Int32::SharedPtr msg);
};