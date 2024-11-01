#ifndef PID_H
#define PID_H

class PID_IMPL; // 前向声明，接口、实现分离

class PID{
public:
    PID(double dt, double max, double min,
        double kp, double ki, double kd);

    ~PID();
    // 计算PID输出
    double calculatePID(double refValue, double realValue);
    
private:
    PID_IMPL* pid_impl;
};

#endif
