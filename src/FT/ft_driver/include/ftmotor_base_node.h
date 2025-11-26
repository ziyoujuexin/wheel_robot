
#include <iostream>
#include "SCServo.h"
#include <termios.h>
#include <unistd.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "ftmsg/msg/motor_data.hpp"
#include <optional>

#define SPE_MODE 1
#define POS_MODE 0

class ftmsg_struct
{
public:
    // ftmsg_struct();
    ftmsg_struct(std::optional<int> id = std::nullopt,std::optional<int> pos = std::nullopt,std::optional<int> pos_spe = std::nullopt,std::optional<int> mode = std::nullopt,std::optional<int> spe = std::nullopt,std::optional<int> acc = std::nullopt);
    void setFs(std::optional<int> id = std::nullopt,std::optional<int> pos = std::nullopt,std::optional<int> pos_spe = std::nullopt,std::optional<int> motor_mode = std::nullopt,std::optional<int> spe = std::nullopt,std::optional<int> acc = std::nullopt);
    ftmsg::msg::MotorData sendFs();

private:
    u8 id = 1;
    s16 position = 0;
    
    u16 pos_speed = 0;
    u8 motor_mode = POS_MODE;
    s16 speed = 0;
    u8 acc = 0;
};

class ftNode : public rclcpp::Node
{
public:
    ftNode();
    ftNode(const char* nodename);
    ~ftNode();
    void init_st();
    void ftPubilshfun();
    void setinfo(ftmsg::msg::MotorData& msg);
    rclcpp:: Publisher<ftmsg::msg::MotorData>::SharedPtr ftPublisher;
    rclcpp:: Subscription<ftmsg::msg::MotorData>::SharedPtr ftSubscription;
    void ftSubCallback(const ftmsg::msg::MotorData::SharedPtr msg);

private:
    SMS_STS st;
    std::string serial = "/dev/ttyCH341USB1";
    ftmsg::msg::MotorData info;
};

