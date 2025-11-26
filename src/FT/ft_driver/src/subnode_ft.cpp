#include "ftmotor_base_node.h"

int main(int argc,char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<ftNode>();
    node->init_st();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 1;
}

ftNode::ftNode() : Node("ftNode")
{
    ftSubscription = this->create_subscription<ftmsg::msg::MotorData>("/info",10,std::bind(&ftNode::ftSubCallback,this,std::placeholders::_1));
    ftPublisher = this->create_publisher<ftmsg::msg::MotorData>("/info",10);
    this->declare_parameter<std::string>("serial_port","/dev/ttyUSB0");
    serial = this->get_parameter("serial_port").as_string();

    std::cout<<"serial:"<<serial<<std::endl;
    // init_st();
    RCLCPP_INFO(this->get_logger(),"ros2 node init finish");
}