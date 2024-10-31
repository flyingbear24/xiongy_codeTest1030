#include "ros/ros.h"
#include "std_msgs/String.h"

// 回调函数
void domsg(const std_msgs::String::ConstPtr &msg_p)
{
    ROS_INFO("我听见：%s", msg_p->data.c_str());
}

int main(int argc, char* argv[])
{
    setlocale(LC_ALL, "");
    // 2-初始化《节点》
    ros::init(argc, argv, "listener_001");
    // 3-实例化句柄
    ros::NodeHandle nh;
    // 4-实例化《订阅者对象》
    ros::Subscriber sub = nh.subscribe<std_msgs::String>("chatter_001",10, domsg);
    //
    // 5-循环调用回调函数
    ros::spin();   // 持续调用ros::spinOnce();

    return 0;
}

/*

std_msgs::String::ConstPtr 
    是一个类型定义，它是一个指向 std_msgs::String 类型消息的只读共享指针。
    ConstPtr 表示指针指向的对象是只读的，不能通过这个指针修改对象的内容。

引用和取地址：
    1.引用在“赋值=的左边“，取地址在赋值=的右边
    2.和“类/函数定义“在一起的是引用，和变量在一起的是取地址
        int &i = a; ---引用  
        int* i = &a;---取地址

引用和指针的区别：
    1.引用不可以为空，但指针可以为空
    2.引用不可以改变指向
    3.引用是别名，引用的大小是变量的大小；指针的大小为指针本身的大小(4字节)
    4.引用更安全，没有空引用；const指针可能为空null；
        野指针：多个指针指向1块内存，free一个指针之后，别的指针成为了野指针

    5.不要返回局部变量的引用
    6.如果函数做左值，需返回引用

*/