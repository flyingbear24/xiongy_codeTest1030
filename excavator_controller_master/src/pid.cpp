#include "pid.h"
#include <cmath>
#include <iostream>

// PID的实现类定义
class PID_IMPL{
public:
    PID_IMPL(double dt, double max, double min,
            double kp, double ki, double kd);

    ~PID_IMPL();

    double calculatePID(double refValue, double realValue);

private:
    double _dt;         // 控制周期
    double _max;
    double _min;
    double _kp;
    double _ki;
    double _kd;
    double _pre_error;  // 上一次偏差
    double _integrate;  // 积分累计
};

// PID的函数实现
PID::PID(double dt, double max, double min,double kp, double ki, double kd){
    pid_impl = new PID_IMPL(dt, max, min, kp, ki, kd);
}

PID::~PID(){
    delete pid_impl;
}

double PID::calculatePID(double refValue, double realValue){
    return pid_impl->calculatePID(refValue, realValue);
}

// PIDImpl的函数实现
PID_IMPL::PID_IMPL(double dt, double max, double min, double kp, double ki, double kd):
    _dt(dt),
    _max(max),
    _min(min),
    _kp(kp),
    _ki(ki),
    _kd(kd),
    _pre_error(0),
    _integrate(0){}

PID_IMPL::~PID_IMPL(){
}

double PID_IMPL::calculatePID(double refValue, double realValue){
    double error = refValue - realValue;

    double p_out = _kp * error;

    _integrate += error * _dt;
    double i_out = _ki * _integrate;

    double differential = (error - _pre_error) / _dt;
    double d_out = _kd * differential;

    double output = p_out + i_out + d_out;  // pid

    // 限制输出最大最小数值
    if(output > _max)
        output = _max;
    else if (output < _min)
        output = _min;

    _pre_error = error; // 保存更新上一次偏差

    return output;
}