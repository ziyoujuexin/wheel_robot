
#include <iostream>
#include "SCServo.h"
#include <termios.h>
#include <unistd.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "ftmsg/msg/motor_data.hpp"
SMS_STS sm_st;


// u8 id[2] = {1,2};
// s16 position[2];
// u16 speed[2] = {1000,500};
// u8 ACC[2] = {50,10};
// class ftNode : public rclcpp::Node
// {
// public:
//     ftNode() : Node("ftNode")
//     {
//         ftSubscription = this->create_subscription<ftmsg::msg::MotorData>("/ftplace",10,std::bind(&ftNode::ftSubCallback,this,std::placeholders::_1));
                
//         // std::cout<<"serial:"<<argv[1]<<std::endl;
//         if(!st.begin(1000000, serial)){
//             std::cout<<"Failed to init sms/sts motor!"<<std::endl;
//         }
//         RCLCPP_INFO(this->get_logger(),"init finish");
//     }
// private:
//     void ftSubCallback(const ftmsg::msg::MotorData::SharedPtr msg);
//     SMS_STS st;
//     rclcpp:: Publisher<std_msgs::msg::Int32>::SharedPtr ftPublisher;
//     rclcpp:: Subscription<ftmsg::msg::MotorData>::SharedPtr ftSubscription;
//     const char *serial = "/dev/ttyUSB0";
// };



int main()
{
    // rclcpp::init(argc,argv);
    // rclcpp::spin(std::make_shared<ftNode>());
    // rclcpp::shutdown();
    const char *serial = "/dev/ttyCH341USB0";

	std::cout<<"serial:"<<serial<<std::endl;
    if(!sm_st.begin(1000000, serial)){
        std::cout<<"Failed to init sms/sts motor!"<<std::endl;
        return 0;
    }
    char ctrl_word;
    bool is_Spe = false;
    bool is_while =true;
    s16 position = 0;
    s16 speed = 0;
    // int id;
    struct termios new_settings;
    struct termios store_settings;
    tcgetattr(STDIN_FILENO,&store_settings);
    new_settings = store_settings;
    new_settings.c_lflag &= (~ICANON);
    new_settings.c_cc[VTIME] = 0;
    tcgetattr(STDIN_FILENO,&store_settings);
    new_settings.c_cc[VMIN] = 1;
    tcsetattr(STDIN_FILENO, TCSANOW, &new_settings);
   // ctrl_word = getchar();
   
    sm_st.protectedForce(1,10);
    sm_st.isOverload(1,true);
    while(is_while)
    {
        if(is_Spe)
        {
            sm_st.WheelMode(1);
            position = 0;
        }
        else 
        {
            sm_st.PosMode(1);
            speed = 0;
        }
        ctrl_word = getchar();
        switch (ctrl_word)
        {

        case 'w':
            position+=50;
            speed+=30;
            if(position>4095) position = 4095;
            if(speed>2400)speed = 2400;
            break;
        
        case 's':
            speed+=-30;
            position-=50;
            if(position<0) position = 0;
            if(speed<-2400) speed = -2400;
            break;
    
        case 'c':
            is_while = false;
            speed = 0;
            break;
        
        case 'x':
            is_Spe =!is_Spe;
            break;

        
        default:
            break;
        }
        // if(ctrl_word == 'w')
        // {
        // }
        // else if(ctrl_word == 's')
        // {
        //     position--;
        // }else if (ctrl_word == 'c')
        // {
        //     // sm_st.WritePosEx(1,0,2000,0);
        //     std::cout<<"keyborad ctrl finish!"<<std::endl;
        //     break;
        // }
        std::cout<<position<<" "<<speed<<std::endl;
        if(is_Spe) sm_st.WriteSpe(1,speed,0);
        else sm_st.WritePosEx(1,position,2000,0);
        // sm_st.WritePosEx(1,)
    }

    sm_st.end();
    return 1;
}

// void ftNode::ftSubCallback(const ftmsg::msg::MotorData::SharedPtr msg)
// {
//     RCLCPP_INFO(this->get_logger(),"get  msg");
//     int data = msg->place;
//     st.WritePosEx(1,data,2000,0);
    
// }