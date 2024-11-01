#ifndef PROJECT_VECTOR2D_H // 防止头文件被重复包含
#define PROJECT_VECTOR2D_H

#include <iostream> // 输入输出流库
#include <cmath> // 数学函数库

// 二维向量类
class VECTOR2D {
public:
    double x, y; // x和y轴上的分量

    // 构造函数，初始化x和y的值，默认为0
    VECTOR2D(double initial_x = 0.0, double initial_y = 0.0) {
        x = initial_x;
        y = initial_y;
    }

    // 计算向量的大小（长度）
    double getMagnitude() {
        return sqrt(x*x + y*y);
    }

    // 计算向量大小的平方，用于避免开方运算，提高性能
    double getMagnitudeSquared() {
        return (x*x + y*y);
    }

    // 重载运算符：向量相减
    VECTOR2D operator-(const VECTOR2D& vec) {
        return VECTOR2D(x - vec.x, y - vec.y);
    }
    // 重载运算符：向量点乘
    double operator*(const VECTOR2D& vec) {
        return x*vec.x + y*vec.y;
    }
    // 重载运算符：向量与标量相乘
    VECTOR2D operator^(double scalar) {
        return VECTOR2D(x*scalar, y*scalar);
    }
    // 重载运算符：向量相加
    VECTOR2D operator+(const VECTOR2D& vec) {
        return VECTOR2D(x + vec.x, y + vec.y);
    }
    // 重载运算符：比较两个向量是否相等
    bool operator==(const VECTOR2D& vec) {
        if (x==vec.x && y==vec.y)
            return true;
        return false;
    }
    // 重载运算符：向量与另一个向量相加并更新自身
    void operator+=(const VECTOR2D& vec) {
        x += vec.x;
        y += vec.y;
    }
};

#endif // PROJECT_VECTOR2D_H


// ===========================================================================


// //
// // Created by kadupitiya on 10/12/18.
// // This is 3d vector class (cartesian coordinate system)
// //

// #ifndef PROJECT_VECTOR2D_H
// #define PROJECT_VECTOR2D_H

// #include <iostream>
// #include<cmath>


// class VECTOR2D
// {

// public:
//     double x, y;								// component along each axis (cartesian)

//     // make a 3d vector
//     VECTOR2D(double initial_x = 0.0, double initial_y = 0.0)
//     {
//         x = initial_x;
//         y = initial_y;
//     }

//     double getMagnitude()								// magnitude of the vector
//     {
//         return sqrt(x*x + y*y);
//     }
//     double getMagnitudeSquared()							// magnitude squared of the vector
//     {
//         return (x*x + y*y);
//     }

//     // overloading -, *, ^, +, ==, +=
//     VECTOR2D operator-(const VECTOR2D& vec)						// subtract two vectors
//     {
//         return VECTOR2D(x - vec.x, y - vec.y);
//     }
//     double operator*(const VECTOR2D& vec)						// dot product of two vectors
//     {
//         return x*vec.x + y*vec.y;
//     }
//     VECTOR2D operator^(double scalar)						// product of a vector and a scalar
//     {
//         return VECTOR2D(x*scalar, y*scalar);
//     }
//     VECTOR2D operator+(const VECTOR2D& vec)						// add two vectors
//     {
//         return VECTOR2D(x + vec.x, y + vec.y);
//     }
//     bool operator==(const VECTOR2D& vec)						// compare two vectors
//     {
//         if (x==vec.x && y==vec.y)
//             return true;
//         return false;
//     }
//     void operator+=(const VECTOR2D& vec)
//     {
//         x += vec.x;
//         y += vec.y;
//     }
// };

// #endif //PROJECT_VECTOR2D_H




