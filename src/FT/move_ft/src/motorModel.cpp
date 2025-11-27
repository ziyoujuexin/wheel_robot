#include "motorModel.h"
//Todo:: wave the arm when running and stop(model 0 model 1)
/*struct
launch begin subscription to control motor
another subscription for change model
model 0:手臂平举/放下√
model 1:轻微摆动（可偏后）√
可加：启动时挥手？√
model n:情绪反应，dance？
*/
int main(int argc,char* argv[])
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<moveNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while (rclcpp::ok())
    {
        executor.spin_some();
    }
    
    // rclcpp::spin(node);
    rclcpp::shutdown();
}

moveNode::moveNode() : ftNode("moveNode")
{
    modelSub = this->create_subscription<std_msgs::msg::Int32>("/model",10,[this](const std_msgs::msg::Int32::SharedPtr msg){modelSubCallBack(msg);});
    otherModeSub = this->create_subscription<std_msgs::msg::Int8>("/mode",10,[this](const std_msgs::msg::Int8::SharedPtr msg){otherModeSubCallBack(msg);});
    cmdSub = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel",10,[this](const geometry_msgs::msg::Twist::SharedPtr msg){cmdSubCallBack(msg);});
    ftPublisher = this->create_publisher<ftmsg::msg::MotorData>("/info",10);
    followSub = this->create_subscription<wheeltec_mic_msg::msg::MotionControl>("/follow_flag",10,[this](const wheeltec_mic_msg::msg::MotionControl::SharedPtr msg){followSubCallBack(msg);});
    timer_ = this->create_wall_timer(std::chrono::milliseconds(10),[this](){state_time_check();});
    msg_ft.setFs(1,2000,1000,POS_MODE,std::nullopt,30);
    this->declare_parameter<std::vector<long int>>("keep_times",std::vector<long int>{1,1,1,1});
    this->declare_parameter<std::vector<long int>>("motorSpeed",std::vector<long int>{1,1,1,1,1});
    auto param_vec = this->get_parameter("keep_times").as_integer_array();
    motorSpeed = this->get_parameter("motorSpeed").as_integer_array();
    for(int i=0; i<5; i++)
    {
        keep_times[i] = static_cast<int>(param_vec[i]);
    }
    RCLCPP_INFO(this->get_logger(),"times:%d",keep_times[4]);  
    // RCLCPP_INFO(this->get_logger(),"times:%ld",param_vec[1]);  
      
}


void moveNode::modelSubCallBack(const std_msgs::msg::Int32::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(),"start excute model%d",msg->data);

    if(msg->data != state && state != CHANGE){
        // is_state_change = true;
        RCLCPP_INFO(this->get_logger(),"state_change");
        current_step = 0;
        // state = CHANGE;
    }
    next_state = msg->data;
}

void moveNode::otherModeSubCallBack(const std_msgs::msg::Int8::SharedPtr msg)
{

}
void moveNode::followSubCallBack(const wheeltec_mic_msg::msg::MotionControl::SharedPtr msg)
{
    if(msg->follow_flag == 1) is_follow = true;
    else is_follow = false;
}
void moveNode::cmdSubCallBack(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    if(msg->linear.x != 0)
    {
        swing_dgree = 300+ fabs(msg->linear.x)*30;
        // state = CHANGE;
        next_state = SWING;

    
    }

}
void moveNode::modelExcute(int step)
{
    if(state != next_state) state = CHANGE;
    int base = 2000;
    RCLCPP_INFO(this->get_logger(),"state:%d",state);
    switch (state)
    {
    case CHANGE:
        state = next_state;

        switch(next_state)
        {
        case STOP:

            break;
        case WAVE:
            sendmsg(HEAD,2048,200,POS_MODE,std::nullopt,10);
            sendmsg(LEFT_SHOULDER,4095,2500,POS_MODE,std::nullopt,30);  
            sendmsg(LEFT_ARM,1275);
            last_time = this->now();

            break;
        case SWING:
            sendmsg(HEAD,2048,200,POS_MODE,std::nullopt,10);
            sendmsg(RIGHT_ARM,1200,1500,POS_MODE,std::nullopt,30);
            sendmsg(LEFT_ARM,1200);

            last_time = this->now();

            break;
        case NOD:
            sendmsg(HEAD,2048,200,POS_MODE,std::nullopt,10);
            break;
        case HAPPY:
            sendmsg(HEAD,1800,500,POS_MODE,std::nullopt,30);
            // sendmsg(LEFT_ARM,900,1500);  //小头版本
            // sendmsg(RIGHT_ARM,900);
            sendmsg(LEFT_ARM,1275);
            sendmsg(RIGHT_ARM,1275);
            sendmsg(LEFT_SHOULDER,3700);
            sendmsg(RIGHT_SHOULDER,300);
            last_time = this->now();
            break;
        
        case SAD:
            // sendmsg(HEAD,2000,500,POS_MODE);
            sendmsg(HEAD,2100,300);
            sendmsg(LEFT_SHOULDER,3100,1500);
            sendmsg(RIGHT_SHOULDER,1000);
            // sendmsg()
            break;
        
        case CONFUSE:
            sendmsg(HEAD,2100,500,std::nullopt,0,30);
            sendmsg(NECK,2300,500);
            sendmsg(LEFT_SHOULDER,3336,1500,std::nullopt,0,50);
            sendmsg(LEFT_ARM,854,1500,std::nullopt,0,50);
            break;

        default:
            
            break;
        }
        break;
    case WAVE:
        switch (step%2)
        {
        case 0:
            sendmsg(LEFT_ARM,1500,1000);
            break;
        case 1:
            sendmsg(LEFT_ARM,1275);
            break;
        default:
            break;
        }


        break;
    case SWING:
        // RCLCPP_INFO(this->get_logger(),"waving...");

        if(step%2 == 0)
        {
            // rclcpp::sleep_for(std::chrono::milliseconds(1000));
            sendmsg(1,base + swing_dgree);
            sendmsg(2,base + swing_dgree);        
        }
        // rclcpp::sleep_for(std::chrono::milliseconds(1000));
        else
        {

            sendmsg(1,base - swing_dgree);
            sendmsg(2,base - swing_dgree);
        }
        // rclcpp::sleep_for(std::chrono::milliseconds(1000));

        
        // setinfo()
        break;
    case STOP:
        sendmsg(LEFT_SHOULDER,2000,1000,POS_MODE,std::nullopt,30);
        sendmsg(RIGHT_SHOULDER,2000);
        sendmsg(RIGHT_ARM,900);
        sendmsg(LEFT_ARM,900);  
        sendmsg(HEAD,2048,900,POS_MODE,std::nullopt,10);
        // sendmsg(NECK,2048,500);
        sendmsg(NECK,2048);
        break;
    
    case SLEEP:

        break;
    case NOD:
        if(step%2 == 0)
        {
            sendmsg(HEAD,1800,200,POS_MODE,std::nullopt,10);    
        }
        else
        {
            sendmsg(HEAD,2100);
        }
        
        break;

    case HAPPY:
        switch (step%2)
        {
        case 0:
            //case: 小头版本
            // sendmsg(LEFT_ARM,1400,1500);
            // sendmsg(RIGHT_ARM,1400);
            
            sendmsg(LEFT_ARM,1500,1500);
            sendmsg(RIGHT_ARM,1500);

            // sendmsg(HEAD,2200,200);
            // sendmsg(LEFT_ARM,1400,500);
            // sendmsg(RIGHT_ARM,1000);
            break;
        case 1:
            // V小头
            // sendmsg(LEFT_ARM,1000,1500);
            // sendmsg(RIGHT_ARM,1000);
            
            sendmsg(LEFT_ARM,1275,1500);
            sendmsg(RIGHT_ARM,1275);
            // sendmsg(HEAD,1800,200);
            break;
        default:
            break;
        }
        
        break;
    case SAD:
        switch (step%2)
        {
        case 0: 

            sendmsg(LEFT_ARM,730,700);
            sendmsg(RIGHT_ARM,730);
            sendmsg(NECK,2200,500);
            break;
        case 1:
            sendmsg(NECK,1800,500);
            break;
        default:
            break;
        }


        break;
    case CONFUSE:
        if(step%2 == 0)
        {
            sendmsg(LEFT_ARM,950,400);
            sendmsg(LEFT_SHOULDER,3403,700);
        }
        else
        {
            sendmsg(LEFT_SHOULDER,3227,700);
            sendmsg(LEFT_ARM,678,400);
        }
        break;
    default:
        break;
        
    }
}

void moveNode::state_time_check()
{
    // auto info = msg_ft.sendFs();
    auto now = this->now();

    if(current_step == 0 && !is_state_init)
    {
        last_time = now;
        is_state_init = true;
    }

    if((now - last_time).seconds() >= get_delay_step(current_step))
    {
        modelExcute(current_step);
        last_time = now;
        current_step++;
    }
    // if((now - last_time).seconds() >= sleepTime)
    // {
    //     state = CHANGE;
    //     next_state = SLEEP;
    //     RCLCPP_INFO(this->get_logger(),"FALL a SLEEP");
    // }
    if(current_step >= keep_times[(state >=0 && state < 4) ? state : 0]&&state!=STOP)
    {
        current_step  = 0;
        next_state = STOP;
        // state = CHANGE;
    }
    if(is_follow == true) next_state = SWING;
}

double moveNode::get_delay_step(int step)//todo:change delay time lode function to hashmap
{
    std::vector<std::vector<double>> delay_time = {
        {1.0,1.0},
        {0.7,0.7},
        {0.8,0.8},
        {1.0,1.0},
        {0.6,0.6},
        {0.8,0.8},
        {1.0,1.0}
    };
    int s = (state >=0 && state <= 6) ? state : 0;
    return delay_time[s][step%delay_time[s].size()];
}

void moveNode::sendmsg(
    std::optional<int> id,
    std::optional<int> pos,
    std::optional<int> pos_spe,
    std::optional<int> motor_mode,
    std::optional<int> spe,
    std::optional<int> acc)
{
    msg_ft.setFs(id,pos,pos_spe,motor_mode,spe,acc);
    auto info = ftmsg::msg::MotorData();
    info = msg_ft.sendFs();
    setinfo(info);
    ftPubilshfun();
}