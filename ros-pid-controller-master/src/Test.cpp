// 引入自定义的PID控制器头文件
#include "pid.h"
// 引入标准输入输出库
#include <stdio.h>
// 引入自定义的几何计算头文件
#include "geometry.h"

// 主函数的替代版本，通常用于测试特定功能
int main2() {
    // 创建一个PID控制器对象，初始化参数为：控制周期(0.1)，输出最大值(100)，输出最小值(-100)，比例增益(0.1)，微分增益(0.01)，积分增益(0.5)
    PID pid = PID(0.1, 100, -100, 0.1, 0.01, 0.5);

    // 初始化一个变量val，用于存储测试的中间值
    double val = 20;
    // 循环100次，模拟PID控制器的计算过程
    for (int i = 0; i < 100; i++) {
        // 使用PID控制器计算增量inc
        double inc = pid.calculate(0, val);
        // 打印当前的val值和增量inc
        printf("val:% 7.3f inc:% 7.3f\n", val, inc);
        // 更新val值
        val += inc;
    }

    // 创建两个二维向量point1和point2，用于表示线段的起点和终点
    VECTOR2D point1(5,5);
    VECTOR2D point2(10,10);
    // 创建一个二维向量point，用于表示需要计算距离的点
    VECTOR2D point(10,5);

    // 创建一个几何计算对象geo
    Geometry geo;
    // 使用geo对象的getLineSegment函数创建一个线段对象line，表示point1和point2之间的线段
    Geometry::LineSegment line = geo.getLineSegment(point1,point2);
    // 使用geo对象的getMinimumDistanceLine函数计算点point到线段line的最小距离线段distance
    Geometry::LineSegment distance = geo.getMinimumDistanceLine(line,point);

    // 使用geo对象的isPointRightOfLine函数判断点point是否在line的右侧，结果是布尔值
    bool pointRight = geo.isPointRightOfLine(line,point);

    // 打印线段distance的最小距离和线段line的梯度（斜率）
    printf("Min Distance:% 7.3f Angle:% 7.3f \n", distance.disatance, line.gradient);

    // 使用标准输出流cout打印点point是否在line的右侧的结果
    cout << "Point Right: " << pointRight <<endl;

    // 返回0，表示程序正常结束
    return 0;
}


// =======================================================================================


// //
// // Created by kadupitiya on 10/10/18.
// //
// #include "pid.h"
// #include <stdio.h>
// #include "geometry.h"

// int main2() {

//     PID pid = PID(0.1, 100, -100, 0.1, 0.01, 0.5);

//     double val = 20;
//     for (int i = 0; i < 100; i++) {
//         double inc = pid.calculate(0, val);
//         printf("val:% 7.3f inc:% 7.3f\n", val, inc);
//         val += inc;
//     }

//     VECTOR2D point1(5,5);
//     VECTOR2D point2(10,10);
//     VECTOR2D point(10,5);

//     Geometry geo;
//     Geometry::LineSegment line = geo.getLineSegment(point1,point2);
//     Geometry::LineSegment distance = geo.getMinimumDistanceLine(line,point);


//     bool pointRight= geo.isPointRightOfLine(line,point);

//     printf("Min Distance:% 7.3f Angle:% 7.3f \n", distance.disatance,line.gradient);

//     cout << "Point Right: " << pointRight <<endl;


//     return 0;

// }