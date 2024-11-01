#include "pid.h"  // 包含PID控制器的头文件
#include <iostream>  // 输入输出流库
#include <cmath>  // 数学函数库

// PID控制器的实现类定义
class PIDImpl {
public:
    // 构造函数，初始化PID控制器的参数
    PIDImpl(double dt, double max, double min, double Kp, double Kd, double Ki);
    // 析构函数
    ~PIDImpl();
    // 计算PID输出
    double calculate(double setPoint, double processValue);

private:
    double _dt;  // 控制周期
    double _max;  // 输出的最大值
    double _min;  // 输出的最小值
    double _Kp;  // 比例增益
    double _Kd;  // 微分增益
    double _Ki;  // 积分增益
    double _pre_error;  // 上一次的偏差
    double _integral;  // 积分累计
};

// PID控制器的构造函数实现
PID::PID(double dt, double max, double min, double Kp, double Kd, double Ki) {
    pimpl = new PIDImpl(dt, max, min, Kp, Kd, Ki);  // 创建PID控制器的实现对象
}

// PID控制器的计算函数实现
double PID::calculate(double setPoint, double processValue) {
    return pimpl->calculate(setPoint, processValue);  // 调用实现对象的计算函数
}

// PID控制器的析构函数实现
PID::~PID() {
    delete pimpl;  // 删除PID控制器的实现对象
}

// PID控制器实现类的构造函数
PIDImpl::PIDImpl(double dt, double max, double min, double Kp, double Kd, double Ki) :
        _dt(dt),
        _max(max),
        _min(min),
        _Kp(Kp),
        _Kd(Kd),
        _Ki(Ki),
        _pre_error(0),
        _integral(0) {
    // 初始化PID控制器的参数
}

/*
 - 比例（Proportional）(P)：计算当前偏差（设定值与实际值的差距），并乘以比例增益（Kp）。
    输出对系统的当前状态进行纠正。

- 积分（Integral)（I）：累积偏差随时间的总和，并乘以积分增益（Ki）。
    解决由于比例控制引入的静态误差，确保系统最终能够达到设定值。

- 微分（Derivative）（D）：计算偏差变化率，并乘以微分增益（Kd）。
    抑制系统的振荡，提高系统的稳定性。
*/

// PID控制器实现类的计算函数
double PIDImpl::calculate(double setPoint, double processValue) {
    double error = setPoint - processValue;  // 计算偏差

    double Pout = _Kp * error;  // 比例项

    _integral += error * _dt;  // 积分项
    double Iout = _Ki * _integral;

    double derivative = (error - _pre_error) / _dt;  // 微分项
    double Dout = _Kd * derivative;

    double output = Pout + Iout + Dout;  // 计算总输出

    if (output > _max)  // 限制输出的最大值
        output = _max;
    else if (output < _min)  // 限制输出的最小值
        output = _min;

    _pre_error = error;  // 更新上一次的偏差

    return output;  // 返回PID输出
}

// PID控制器实现类的析构函数
PIDImpl::~PIDImpl() {
    // 清理资源
}


// ====================================================================================


// #include "pid.h"
// #include <iostream>
// #include <cmath>

// class PIDImpl {
// public:
//     PIDImpl(double dt, double max, double min, double Kp, double Kd, double Ki);
//     ~PIDImpl();
//     double calculate(double setPoint, double processValue);

// private:
//     double _dt;
//     double _max;
//     double _min;
//     double _Kp;
//     double _Kd;
//     double _Ki;
//     double _pre_error;
//     double _integral;
// };

// PID::PID(double dt, double max, double min, double Kp, double Kd, double Ki) {
//     pimpl = new PIDImpl(dt, max, min, Kp, Kd, Ki);
// }

// double PID::calculate(double setPoint, double processValue) {
//     return pimpl->calculate(setPoint, processValue);
// }

// PID::~PID() {
//     delete pimpl;
// }

// PIDImpl::PIDImpl(double dt, double max, double min, double Kp, double Kd, double Ki) :
//         _dt(dt),
//         _max(max),
//         _min(min),
//         _Kp(Kp),
//         _Kd(Kd),
//         _Ki(Ki),
//         _pre_error(0),
//         _integral(0) {
// }


// /*
//  - 比例（Proportional）(P)：

//     计算当前偏差（设定值与实际值的差距），并乘以比例增益（Kp）。
//     输出对系统的当前状态进行纠正。

// - 积分（Integral)（I）：

//     累积偏差随时间的总和，并乘以积分增益（Ki）。
//     解决由于比例控制引入的静态误差，确保系统最终能够达到设定值。

// - 微分（Derivative）（D）：

//     计算偏差变化率，并乘以微分增益（Kd）。
//     抑制系统的振荡，提高系统的稳定性。
// */
// double PIDImpl::calculate(double setPoint, double processValue) {
//     double error = setPoint - processValue;

//     double Pout = _Kp * error;

//     _integral += error * _dt;
//     double Iout = _Ki * _integral;

//     double derivative = (error - _pre_error) / _dt;
//     double Dout = _Kd * derivative;

//     double output = Pout + Iout + Dout;

//     if (output > _max)
//         output = _max;
//     else if (output < _min)
//         output = _min;

//     _pre_error = error;

//     return output;
// }

// PIDImpl::~PIDImpl() {
// }
