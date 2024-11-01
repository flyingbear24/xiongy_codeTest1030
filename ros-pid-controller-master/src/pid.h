#ifndef PID_H // 防止头文件被重复包含
#define PID_H

// 前向声明，隐藏细节，接口实现分离
class PIDImpl;

// PID控制器类声明
class PID {
public:
    // 构造函数，初始化PID控制器
    PID(double dt, double max, double min, double Kp, double Kd, double Ki);
    // 计算PID输出
    double calculate(double setPoint, double processValue);
    // 析构函数
    ~PID();

private:
    PIDImpl* pimpl; // 指向实现类的指针
};

#endif // PID_H


// ================================================================================


// #ifndef PID_H
// #define PID_H

// class PIDImpl;

// class PID {
// public:
//     PID(double dt, double max, double min, double Kp, double Kd, double Ki);
//     double calculate(double setPoint, double processValue);
//     ~PID();

// private:
//     PIDImpl* pimpl;
// };

// #endif //PID_H
