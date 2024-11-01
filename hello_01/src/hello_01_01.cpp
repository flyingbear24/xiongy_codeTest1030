#include "ros/ros.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "hello");     // ros节点初始化
    ros::NodeHandle n;  // 创建ros节点句柄（非必须）
    ROS_INFO("hello_hahaha");   // 控制台输出


    return 0;
}