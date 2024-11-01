# include "ros/ros.h"
# include "pid.h"
# include "std_msgs/Float64.h"
# include <cmath>

// 全局变量
ros::Subscriber x_sub;
ros::Publisher x_ref_pub;
ros::Publisher u_pub;
double x_angularVelocity = 0;
double x_angularVelocity_ref = 0;
double u_joystick = 0;

std_msgs::Float64 msg_joystick;
std_msgs::Float64 msg_ref_velocty;

// pid参数
double dt = 0.1, max = 1000, min = -1000,
        kp = 250, ki = 100, kd = 10;
PID pidController(dt, max, min, kp, ki, kd);
        
// 订阅角速度回调函数
void x_Callback(const std_msgs::Float64::ConstPtr &x_sub)
{
    x_angularVelocity = x_sub->data;
    ROS_INFO("实际角速度为: %f", x_sub->data);
}

// 参考轨迹
double refPoint_create(int &count)
{
    return 20 * sin(count*M_PI/175) * sin(count*M_PI/40);
}

int main(int argc, char** argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv , "CONTROLLER_MASTER");
    ros::NodeHandle nh;

    x_sub = nh.subscribe<std_msgs::Float64>("X_VELOCITY", 10, x_Callback);  // 订阅

    int count = 1;
    ros::Rate rt(10);
    while (ros::ok())
    {
        x_angularVelocity_ref = refPoint_create(count);
        u_joystick = pidController.calculatePID(x_angularVelocity_ref, x_angularVelocity);

        u_pub = nh.advertise<std_msgs::Float64>("U_JOYSTICK", 10);
        msg_joystick.data = u_joystick;
        u_pub.publish(msg_joystick);    // 发布手柄信号

        x_ref_pub = nh.advertise<std_msgs::Float64>("X_REF_VELOCITY", 10);
        msg_ref_velocty.data = x_angularVelocity_ref;
        x_ref_pub.publish(msg_ref_velocty);    // 发布参考轨迹
        ROS_INFO("参考角速度为： %f", x_angularVelocity_ref);    

        count++;
        rt.sleep();
        ros::spinOnce();
    }
    return 0;
}