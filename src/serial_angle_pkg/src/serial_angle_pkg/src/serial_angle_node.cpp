#include "rclcpp/rclcpp.hpp"
#include <serial/serial.h>
#include <memory>
#include <thread>
#include <vector>
#include <string>
#include <sstream>
#include <regex>
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int32.hpp"

class SerialAngleNode : public rclcpp::Node  // 确保类名正确
{
public:
    SerialAngleNode() : Node("serial_angle_node")  // 确保节点名正确
    {
        // 首先列出所有可用串口
        list_available_ports();
        
        // 尝试多个可能的设备
        std::vector<std::string> possible_ports = {
            "/dev/ttyCH343USB1","/dev/ttyCH343USB0",
            "/dev/ttyCH343USB2"
        };
        
        bool port_opened = false;
        std::string error_messages;
        
        for (const auto& port : possible_ports) {
            try {
                RCLCPP_INFO(this->get_logger(), "尝试打开串口: %s", port.c_str());
                
                // 检查设备是否存在
                if (access(port.c_str(), F_OK) == -1) {
                    RCLCPP_WARN(this->get_logger(), "设备不存在: %s", port.c_str());
                    continue;
                }
                
                // 检查读写权限
                if (access(port.c_str(), R_OK | W_OK) == -1) {
                    RCLCPP_WARN(this->get_logger(), "设备无读写权限: %s", port.c_str());
                    // 尝试修改权限（需要sudo权限）
                    std::string cmd = "sudo chmod 666 " + port;
                    system(cmd.c_str());
                }
                
                serial_port.setPort(port);
                serial_port.setBaudrate(115200);
                serial_port.setBytesize(serial::eightbits);
                serial_port.setParity(serial::parity_none);
                serial_port.setStopbits(serial::stopbits_one);
                serial_port.setFlowcontrol(serial::flowcontrol_none);
                
                serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
                serial_port.setTimeout(timeout);
                
                serial_port.open();
                
                // 验证串口确实打开
                if (serial_port.isOpen()) {
                    RCLCPP_INFO(this->get_logger(), "串口打开成功: %s", port.c_str());
                    port_opened = true;
                    break;
                } else {
                    RCLCPP_WARN(this->get_logger(), "串口打开但验证失败: %s", port.c_str());
                }
                
            } catch (const std::exception& e) {
                std::string error_msg = "无法打开串口 " + port + ": " + e.what() + "\n";
                error_messages += error_msg;
                RCLCPP_ERROR(this->get_logger(), "%s", error_msg.c_str());
            }
        }
        
        if (!port_opened) {
            RCLCPP_ERROR(this->get_logger(), "所有串口打开尝试都失败了");
            RCLCPP_ERROR(this->get_logger(), "错误汇总:\n%s", error_messages.c_str());
            // 不退出，让节点继续运行但无法读取串口
        }
        
        // 创建定时器读取串口数据 - 确保使用正确的类名
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&SerialAngleNode::readSerialData, this));  // 这里改为 SerialAngleNode
            
        // 创建发布者 - 发布到 awake_angle 话题，使用 UInt32 类型
        angle_publisher_ = this->create_publisher<std_msgs::msg::UInt32>("awake_angle", 10);
        
        // 可选：保留原来的字符串发布者用于调试
        debug_publisher_ = this->create_publisher<std_msgs::msg::String>("serial_data_debug", 10);
    }
    
    ~SerialAngleNode() {  // 确保析构函数名正确
        if (serial_port.isOpen()) {
            serial_port.close();
            RCLCPP_INFO(this->get_logger(), "串口已关闭");
        }
    }

private:
    void list_available_ports()
    {
        RCLCPP_INFO(this->get_logger(), "搜索可用的串口设备...");
        try {
            std::vector<serial::PortInfo> devices_found = serial::list_ports();
            
            if (devices_found.empty()) {
                RCLCPP_INFO(this->get_logger(), "未找到任何串口设备");
            } else {
                for (const auto& device : devices_found) {
                    RCLCPP_INFO(this->get_logger(), "找到设备: %s - %s [%s]", 
                               device.port.c_str(), 
                               device.description.c_str(),
                               device.hardware_id.c_str());
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "搜索串口设备时出错: %s", e.what());
        }
        
        // 同时检查常见的设备文件
        RCLCPP_INFO(this->get_logger(), "检查常见设备文件:");
        std::vector<std::string> common_ports = {
            "/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2",
            "/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2",
            "/dev/ttyS0", "/dev/ttyS1", "/dev/ttyS2"
        };
        
        for (const auto& port : common_ports) {
            if (access(port.c_str(), F_OK) != -1) {
                // 检查权限
                std::string permissions = "存在";
                if (access(port.c_str(), R_OK) != -1 && access(port.c_str(), W_OK) != -1) {
                    permissions += " (可读写)";
                } else {
                    permissions += " (权限不足)";
                }
                RCLCPP_INFO(this->get_logger(), "  %s: %s", port.c_str(), permissions.c_str());
            }
        }
    }
    
    void readSerialData()
    {
        if (serial_port.isOpen()) {
            try {
                // 检查是否有数据可读
                if (serial_port.available()) {
                    size_t bytes_available = serial_port.available();
                    RCLCPP_DEBUG(this->get_logger(), "有 %zu 字节数据可读", bytes_available);
                    
                    // 读取数据
                    std::string data = serial_port.read(bytes_available);
                    
                    // 处理接收到的数据
                    if (!data.empty()) {
                        RCLCPP_INFO(this->get_logger(), "接收到串口数据: %s", data.c_str());
                        
                        // 尝试将数据转换为角度值
                        uint32_t angle = parseAngleFromSerialData(data);
                        
                        // 只有当成功解析到有效角度时才发布
                        if (angle != 9999) { // 9999 是我们的错误代码
                            // 发布角度数据到 awake_angle 话题
                            auto angle_msg = std_msgs::msg::UInt32();
                            angle_msg.data = angle;
                            angle_publisher_->publish(angle_msg);
                            RCLCPP_INFO(this->get_logger(), "发布角度数据: %u", angle);
                        }
                        
                        // 可选：发布原始数据用于调试
                        auto debug_msg = std_msgs::msg::String();
                        debug_msg.data = "原始数据: " + data;
                        debug_publisher_->publish(debug_msg);
                    }
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "串口读写错误: %s", e.what());
            }
        }
    }
    
    // 解析串口数据，提取角度值
    uint32_t parseAngleFromSerialData(const std::string& data)
    {
        // 根据您的数据格式，我们需要从类似这样的字符串中提取角度：
        // "I (3205639) main: Sound event processed. Detected Angle: 331, Confidence: 1.86"
        
        // 方法1：使用正则表达式匹配
        std::regex angle_pattern("Detected Angle:\\s*(\\d+)");
        std::smatch matches;
        
        if (std::regex_search(data, matches, angle_pattern)) {
            if (matches.size() > 1) {
                try {
                    std::string angle_str = matches[1].str();
                    uint32_t angle = std::stoul(angle_str);
                    RCLCPP_INFO(this->get_logger(), "成功解析角度: %u", angle);
                    return angle;
                } catch (const std::exception& e) {
                    RCLCPP_WARN(this->get_logger(), "角度值转换失败: %s", e.what());
                }
            }
        }
        
        // 方法2：使用字符串查找（备用方法）
        size_t angle_pos = data.find("Detected Angle:");
        if (angle_pos != std::string::npos) {
            // 找到 "Detected Angle:" 后，查找后面的数字
            size_t num_start = data.find_first_of("0123456789", angle_pos);
            if (num_start != std::string::npos) {
                size_t num_end = data.find_first_not_of("0123456789", num_start);
                if (num_end != std::string::npos) {
                    try {
                        std::string angle_str = data.substr(num_start, num_end - num_start);
                        uint32_t angle = std::stoul(angle_str);
                        RCLCPP_INFO(this->get_logger(), "成功解析角度: %u", angle);
                        return angle;
                    } catch (const std::exception& e) {
                        RCLCPP_WARN(this->get_logger(), "角度值转换失败: %s", e.what());
                    }
                }
            }
        }
        
        // 如果无法解析，返回错误代码
        RCLCPP_WARN(this->get_logger(), "无法从数据中解析角度: %s", data.c_str());
        return 9999; // 使用特殊值表示解析失败
    }
    
    serial::Serial serial_port;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr angle_publisher_;  // 角度发布者
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_publisher_;  // 调试发布者（可选）
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialAngleNode>();  // 确保使用正确的类名
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
