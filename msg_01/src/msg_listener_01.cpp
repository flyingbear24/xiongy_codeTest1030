#include "ros/ros.h"
#include "msg_01/Person.h"

void domsg_person(const msg_01::Person::ConstPtr &mp)
// void domsg_person(const msg01::Person &mp)
{
    ROS_INFO("---姓名：%s, 今年%d岁, 身高%.2f米", mp->name.c_str(), mp->age, mp->height);
}

int main(int argc, char* argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "listener_001");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe<msg_01::Person>("chatter_001",10,domsg_person);

    ros::spin();

    return 0;
}
