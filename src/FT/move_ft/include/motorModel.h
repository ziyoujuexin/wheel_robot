#include"ftmotor_base_node.h"
#include <chrono>
#include <vector>
#include "std_msgs/msg/int8.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"
#include "wheeltec_mic_msg/msg/motion_control.hpp"
//动作组定义
#define NOD 3//点头
#define WAVE 2//挥手
#define SWING 1//摆臂
#define STOP 0//手臂归位
#define CHANGE -1//状态间的衔接
#define HAPPY 4
#define CONFUSE 5
#define SAD 6
#define NORMAL 7
#define SLEEP -2 //待机休眠状态
//舵机的ID定义
#define RIGHT_SHOULDER 1
#define RIGHT_ARM 3
#define LEFT_SHOULDER 2
#define LEFT_ARM 4
#define HEAD 5
#define NECK 6
//arm舵机的角度范围
#define ARM_MAX_ANGLE 3300
#define ARM_MIN_ANGLE 900
class moveNode :public ftNode::ftNode
{
public:
    moveNode();
    ~moveNode();
    void sendMove(const ftmsg_struct msg);
    void sendmsg(std::optional<int> id = std::nullopt,std::optional<int> pos = std::nullopt,std::optional<int> pos_spe = std::nullopt,std::optional<int> motor_mode = std::nullopt,std::optional<int> spe = std::nullopt,std::optional<int> acc = std::nullopt);

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr modelSub;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr otherModeSub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdSub;
    rclcpp::Subscription<wheeltec_mic_msg::msg::MotionControl>::SharedPtr followSub;
    // rclcpp::Publisher<ftmsg::msg::MotorData>::SharedPtr ftPub;
    ftmsg_struct msg_ft;
    void otherModeSubCallBack(const std_msgs::msg::Int8::SharedPtr msg);
    void modelSubCallBack(const std_msgs::msg::Int32::SharedPtr msg);
    void cmdSubCallBack(const geometry_msgs::msg::Twist::SharedPtr msg);
    void followSubCallBack(const wheeltec_mic_msg::msg::MotionControl::SharedPtr msg);
    void state_time_check();
    double get_delay_step(int step);
    void modelExcute(int step);
    std::vector<int>keep_times = {0,1,1,1,1};
    int state = STOP,next_state = STOP;
    double delay_time = 0;
    int current_step = 0;
    bool is_state_init = false;
    bool is_state_change =false;
    bool is_follow = false;
    rclcpp::Time last_time;
    double sleepTime = 10.0;
    int swing_dgree = 200;
    std::vector<long int> motorSpeed;
};
