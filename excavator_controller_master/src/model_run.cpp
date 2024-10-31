#include "ros/ros.h"
#include "model.h"
#include <vector>
#include "std_msgs/Float64.h"
using namespace std;

// 全局变量
ros::Subscriber u_sub;
ros::Publisher x_pub;
double u_joystick = 0;
double w_midVarible;
double x_angularVelocity;
double pre_x = 0;

vector<double> vec_model;
std_msgs::Float64 msg_velocity;

// 模型参数
double dt = 0.1, w_max = 21, w_min = -21,
u_bp_l = -200, u_bp_r = 200, w_bp_l = -1, w_bp_r = 1; // 
MODEL model_JoystickToAngular(dt, w_max, w_min, u_bp_l, u_bp_r, w_bp_l, w_bp_r);

// 订阅手柄信号回调函数
void u_Callback(const std_msgs::Float64::ConstPtr &u_sub)
{
    u_joystick = u_sub->data;
    ROS_INFO("接收的手柄信号为: %f", u_sub->data);
}

// 模型运行
void model_run(double &u_joystick, double &pre_x, vector<double> vec_model, MODEL* model)
{
    if(u_joystick > u_bp_r)
        w_midVarible = vec_model[0]*u_joystick + vec_model[1];
    else if(u_joystick >= u_bp_l)
        w_midVarible = vec_model[2]*u_joystick;
    else
        w_midVarible = vec_model[3]*u_joystick + vec_model[4];
    // w_midVarible = u_joystick* 21/1000;

    x_angularVelocity = model->_A*pre_x + model->_B*w_midVarible;

    pre_x = x_angularVelocity;  // 记录更新
}

int main(int argc, char** argv)
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "MODEL_RUN");
    ros::NodeHandle nh;

    vec_model = model_JoystickToAngular.caculateModel();

    u_sub = nh.subscribe<std_msgs::Float64>("U_JOYSTICK", 10, u_Callback);  // 订阅

    ros::Rate rt(10);
    while(ros::ok())
    {
        model_run(u_joystick, pre_x, vec_model, & model_JoystickToAngular);

        x_pub = nh.advertise<std_msgs::Float64>("X_VELOCITY", 10);
        msg_velocity.data = x_angularVelocity;
        x_pub.publish(msg_velocity);                        // 发布
        ROS_INFO("输出角速度为：%f", msg_velocity.data);
    
        ros::spinOnce();
        rt.sleep();
    }

    return 0;
}

