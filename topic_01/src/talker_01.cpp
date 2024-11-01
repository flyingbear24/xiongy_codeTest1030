// 1-包含头文件
#include "ros/ros.h"
#include "std_msgs/String.h"  // 普通文本类型
#include <sstream>

int main(int argc, char* argv[])
{
    setlocale(LC_ALL, "");      // 设置编码

    // 2-初始化《ros节点》
    ros::init(argc, argv, "talker_001");
    // 3-实例化句柄
    ros::NodeHandle nh;
    // 4-实例化《发布者》对象，队列中最大保存的消息数10
    ros::Publisher pub = nh.advertise<std_msgs::String>("chatter_001",10);
    // 5-发布的数据
    std_msgs::String msg_str;
    std::string msg_front = "hello 你好"; // 消息前缀

    int count = 0;
    ros::Rate r(1);  //逻辑(一秒1次)
    while(ros::ok())
    {
        // 使用stringstream拼接字符串和编号：字符串流类
        std::stringstream ss;
        ss << msg_front << count;
        msg_str.data = ss.str();  // ss.str()将ss转化为字符串

        // 发布消息
        pub.publish(msg_str);
        // 加入调试，打印发送的消息
        ROS_INFO("发布的消息：%s", msg_str.data.c_str());

        // 根据前面制定的发送频率自动休眠， 休眠时间 = 1/频率
        r.sleep();
        count++;

        ros::spinOnce();    // 处理回调队列中的所有回调，然后返回
    }
    return 0;
}


/*
std_msgs::String
    是ROS中的标准消息类型之一，用于在不同的节点之间传递字符串数据。
    它是一个简单的消息类型，只包含一个字段 data，用于存储字符串。
    # std_msgs/String.msg
    string data

std::string 
    是C++标准库中的一个类，提供了丰富的方法和操作来处理字符串。
    它支持各种字符串操作，如查找、替换、比较、子串提取等
    #include <string>
    std::string str = "Hello, World!";
    std::string part = str.substr(0, 5); // "Hello"

msg_str 是一个 std_msgs::String 类型的消息，它有一个 data 成员，用于存储字符串数据。
data.c_str() 将 data 转换为C风格的字符串，然后 ROS_INFO 宏将这个字符串打印到日志中。


*/