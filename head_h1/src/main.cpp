#include "ros/ros.h"
#include "hahaha.h"

// namespace hahaha_ns
// {
//     my::my(int m_count)         // 构造函数，加作用域 my::my
//     {
//         this->count = m_count;
//     }
//     void my::run()
//     {
//         ROS_INFO("hello! 会飞的熊 %d", count);
//         count += 1;
//     }
// } // namespace name

int main(int argc, char* argv[])
{
    setlocale(LC_ALL, "");              // 中文格式
    ros::init(argc, argv, "haha_3");    // 初始化节点
    ros::NodeHandle nh;                 // 调用时间函数，必须初始化句柄，否则API调用失败
    
    // hello_ns::my myclass_h;             // 实例化类对象
    // myclass_h(1);            // 错误
    // 构造函数需在申明对象时初始化
    hahaha_ns::my myclass_h;             // 实例化类对象，调用构造函数初始化

    ros::Rate r(1);

    while(ros::ok())
    {
        myclass_h.run();                // 函数调用

        r.sleep();
        ros::spinOnce();  
    }
    return 0;
}