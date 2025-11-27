#include "screen_control.h"
// UartProtocol 配置


// serial::Serial ser;

void ScreenNode::send_frame(uint8_t data_type, const std::vector<uint8_t>& payload) {
    size_t payload_len = payload.size();

    // 构建帧头
    std::vector<uint8_t> frame;
    frame.push_back((FRAME_HEADER >> 8) & 0xFF);
    frame.push_back(FRAME_HEADER & 0xFF);

    // 构建ID、数据类型、长度
    frame.push_back(SENDER_ID);
    frame.push_back(data_type);
    frame.push_back(static_cast<uint8_t>(payload_len));

    // 添加数据负载
    frame.insert(frame.end(), payload.begin(), payload.end());

    // 计算校验和
    uint8_t checksum = 0;
    for (size_t i = 0; i < frame.size(); ++i) {
        checksum += frame[i];
    }
    frame.push_back(checksum & 0xFF);

    // 添加帧尾
    frame.push_back(FRAME_TAIL);

    // 发送数据
    ser.write(frame);
    std::cout << "发送帧: ";
    for (auto byte : frame) {
        printf("%02X ", byte);
    }
    std::cout << std::endl;
}

void ScreenNode::play_animation(const std::string& anim_name) {
    std::vector<uint8_t> payload;
    payload.push_back(0xD0);  // 动画播放标识
    payload.insert(payload.end(), anim_name.begin(), anim_name.end());

    send_frame(DATA_TYPE_MOTION, payload);
}

ScreenNode::ScreenNode(const char* nodename):Node(nodename)
{
    modelSub = this->create_subscription<std_msgs::msg::Int32>("model",10,[this](const std_msgs::msg::Int32::SharedPtr msg){modelSubCallback(msg);});
    std::string port = "/dev/ttyCH341USB1";  // 串口设备路径，根据实际情况修改
    ser.setPort(port);
    ser.setBaudrate(115200);  // 波特率，根据硬件配置选择
    // ser.setTimeout(serial::Timeout::simpleTimeout(1000)); // 超时时间为1秒
    ser.open();
    if (ser.isOpen()) {
        std::cout << "串口 " << port << " 已成功打开！" << std::endl;
    } else {
        std::cerr << "无法打开串口 " << port << "。" << std::endl;
        return;
    }

}

void ScreenNode::modelSubCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    int data = msg->data;
    switch (data)
    {
    case HAPPY:
        play_animation(animation[0]);
        break;
    case CONFUSE:
        play_animation(animation[1]);
        break;
    case SAD:
        play_animation(animation[2]);
        break;
    default:
        break;
    }
}


int main(int argc,char* argv[]) {

    rclcpp::init(argc, argv);
    auto node = std::make_shared<ScreenNode>("screenNode");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while (rclcpp::ok())
    {
        executor.spin_some();
    }
    
    return 0;
}
