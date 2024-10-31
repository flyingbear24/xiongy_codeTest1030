#include "ros/ros.h"
#include "msg_01/Person.h"

int main(int argc, char* argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "talker_001");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<msg_01::Person>("chatter_001",10);

    // 组织被发布的消息
    msg_01::Person mp;
    mp.name = "孙悟空";
    mp.age = 500;
    mp.height = 1.5;

    ros::Rate r(1);
    while(ros::ok())
    {
        pub.publish(mp);
        mp.age += 1;
        ROS_INFO("我叫：%s, 今年%d岁 , 身高%.2f米", mp.name.c_str(), mp.age, mp.height);

        r.sleep();
        ros::spinOnce();
    }
    return 0;
}

