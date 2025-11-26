#include "ftmotor_base_node.h"
// #msg/MotorData
// uint8 id
// uint8 is_spe_mode
// int16 position
// uint16 pos_mode_speed
// int16 speed
// uint8 acc


// u8 id[2] = {1,2};
// s16 position[2];
// u16 speed[2] = {1000,500};
// u8 ACC[2] = {50,10};


void ftNode::ftSubCallback(const ftmsg::msg::MotorData::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(),"get  msg");
    this->info = *msg;
    if(info.is_spe_mode == POS_MODE)
    {
        st.PosMode(info.id);
        st.WritePosEx(info.id,info.position,info.pos_mode_speed,info.acc);
    }
    else
    {
        st.WheelMode(info.id);
        st.WriteSpe(info.id,info.speed,info.acc);
    }
    
    // st.WritePosEx(1,info.position,2000,0);
    
}
ftmsg_struct::ftmsg_struct(std::optional<int> id_opt,
                           std::optional<int> pos_opt,
                           std::optional<int> pos_spe_opt,
                           std::optional<int> mode_opt,
                           std::optional<int> spe_opt,
                           std::optional<int> acc_opt)
{
    id = id_opt.value_or(this->id);
    position = pos_opt.value_or(this->position);
    pos_speed = pos_spe_opt.value_or(this->pos_speed);
    motor_mode = mode_opt.value_or(this->motor_mode);
    speed = spe_opt.value_or(this->speed);
    acc = acc_opt.value_or(this->acc);
}    
void ftmsg_struct::setFs(
    std::optional<int> id_opt,
    std::optional<int> pos_opt,
    std::optional<int> pos_spe_opt,
    std::optional<int> motor_mode_opt,
    std::optional<int> spe_opt,
    std::optional<int> acc_opt)
{
    ftmsg::msg::MotorData msg;
    id = id_opt.value_or(this->id);
    position = pos_opt.value_or(this->position);
    pos_speed = pos_spe_opt.value_or(this->pos_speed);
    motor_mode = motor_mode_opt.value_or(this->motor_mode);
    speed = spe_opt.value_or(this->speed);
    acc = acc_opt.value_or(this->acc);

}

ftmsg::msg::MotorData ftmsg_struct::sendFs()
{
    ftmsg::msg::MotorData msg;
    msg.id = id;
    msg.position = position;
    msg.pos_mode_speed = pos_speed;
    msg.is_spe_mode = motor_mode;
    msg.speed = speed;
    msg.acc = acc;
    return msg;
}

ftNode::ftNode() : Node("ftNode")
{

    // std::cout<<"serial:"<<argv[1]<<std::endl;
    // init_st();
    RCLCPP_INFO(this->get_logger(),"ros2 node init finish");
}
ftNode::ftNode(const char* nodename) : Node(nodename)
{
    RCLCPP_INFO(this->get_logger(),"ros2 node init finish");
}

ftNode::~ftNode() 
{
    st.end();
    RCLCPP_INFO(this->get_logger(),"node end");
}

void ftNode::ftPubilshfun()
{
    ftPublisher->publish(info);
    // RCLCPP_INFO(this->get_logger(),"send msg");
}

void ftNode::setinfo(ftmsg::msg::MotorData& msg)
{
    info = msg;
}

void ftNode::init_st()
{
    RCLCPP_INFO(this->get_logger(),"serial:%s",serial.c_str());
    if(!st.begin(1000000, serial.c_str())){
        std::cout<<"Failed to init sms/sts motor!"<<std::endl;
    }

}