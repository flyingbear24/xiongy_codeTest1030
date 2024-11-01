#ifndef MODEL_H
#define MODEL_H

#include <vector>
using namespace std; 

// 挖掘机手柄-角速度模型
class MODEL{
public:
    MODEL(double dt, double w_max, double w_min,
        double u_bp_l, double u_bp_r, double w_bp_l, double w_bp_r):
        _dt(dt),
        _u_max(1000),
        _u_min(-1000),
        _w_max(w_max),
        _w_min(w_min),
        _u_breakpoint_left(u_bp_l),
        _u_breakpoint_right(u_bp_r),
        _w_breakpoint_left(w_bp_l),
        _w_breakpoint_right(w_bp_r),
        _A(0.9),
        _B(0.1){}

    ~MODEL(){}

    // 计算非线性环节系数
    vector<double> caculateModel(){
        vector<double> vec_model;
        double c = (_w_breakpoint_right - _w_breakpoint_left) / 
                    (_u_breakpoint_right - _u_breakpoint_left);
        double a = (_w_max - _w_breakpoint_right) / 
                    (_u_max - _u_breakpoint_right);
        double d = (_w_breakpoint_left - _w_min) / 
                    (_u_breakpoint_left - _u_min);
        double b = _w_max - a * _u_max ;
        double e = _w_min - d * _u_min ;
        // double a=0,b=0,c=0,d=0,e=0;

        vec_model.push_back(a);
        vec_model.push_back(b);
        vec_model.push_back(c);
        vec_model.push_back(d);
        vec_model.push_back(e);

        return vec_model;
    }

// private:
    double _dt;
    double _u_max;
    double _u_min;
    double _w_max;
    double _w_min;
    double _u_breakpoint_left;
    double _u_breakpoint_right;
    double _w_breakpoint_left;
    double _w_breakpoint_right;
    double _A;
    double _B;
};

#endif