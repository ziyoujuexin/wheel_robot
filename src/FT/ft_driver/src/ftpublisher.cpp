#include "ftmotor_base_node.h"
void inputInfomation(ftmsg_struct& creator,std::shared_ptr<ftNode> node);

// #include "aa.h"
int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<ftNode>();
    ftmsg_struct msg_creator;

    inputInfomation(msg_creator,node);   
    rclcpp::shutdown();
}

void inputInfomation(ftmsg_struct& creator,std::shared_ptr<ftNode> node)
{
    int id, pos, pos_spe, motor_mode, spe, acc;

    std::cout << "\n=== 手动输入电机参数 ===" << std::endl;
    std::cout << "输入电机模式: ";     std::cin >> motor_mode;
    std::cout << "输入ID: ";           std::cin >> id;
    while (rclcpp::ok())
    {
        // if(motor_mode == POS_MODE)
        // {

            
        // }
        // else if(motor_mode == SPE_MODE)
        // {
        //     std::cout << "输入速度: ";         std::cin >> spe;
        // }
        switch (motor_mode)
        {
        case POS_MODE:
            std::cout << "输入位置: ";         std::cin >> pos;
            std::cout << "输入位置速度: ";     std::cin >> pos_spe;
            break;

        case SPE_MODE:
            std::cout<< "输入速度";    std::cin>>spe;
            break;
        
        default:

            break;
        }
        std::cout << "输入加速度: ";       std::cin >> acc;

        creator.setFs(id, pos, pos_spe, motor_mode, spe, acc);
        // creator.sendFs();
        ftmsg::msg::MotorData msg = creator.sendFs();
        node->setinfo(msg);
        node->ftPubilshfun();

        rclcpp::sleep_for(std::chrono::milliseconds(10));

    }

    
}
