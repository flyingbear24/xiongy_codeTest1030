#ifndef GEOMETRY_H // 防止头文件被重复包含
#define GEOMETRY_H

#include <geometry_msgs/Point.h> // ROS几何消息类型
#include <geometry_msgs/Quaternion.h>
#include "vector2D.h" // 包含自定义的二维向量类

using namespace std; // 使用标准命名空间
using namespace geometry_msgs; // 使用geometry_msgs命名空间

// 几何计算类
class Geometry {
public:
    struct LineSegment { // 线段结构
        VECTOR2D startP; // 起点
        VECTOR2D endP; // 终点
        double gradient; // 线段的梯度
        double disatance; // 线段到某点的距离
        double disatanceSquared; // 距离的平方
        double disatanceToAObj; // 到某物体的距离
        LineSegment *shortestDistLine; // 最短距离线段指针
    };

    vector<VECTOR2D> trajectory; // 轨迹点集合
    vector<LineSegment> path; // 路径线段集合

    map<double, VECTOR2D*> distances; // 距离映射
    map<double, LineSegment*> distancesPath; // 路径距离映射

    // 成员函数
    bool checkInsideRectSimple(VECTOR2D &, VECTOR2D &, VECTOR2D &, VECTOR2D &, VECTOR2D &);
    double crossProduct(VECTOR2D &, VECTOR2D &);
    double dotProduct(VECTOR2D &, VECTOR2D &);
    bool doBoundingBoxesIntersect(LineSegment &, LineSegment &);
    bool isPointOnLine(LineSegment &, VECTOR2D &);
    bool isPointRightOfLine(LineSegment &, VECTOR2D &);
    bool lineSegmentTouchesOrCrossesLine(LineSegment &a, LineSegment &b);
    LineSegment getBoundingBox(LineSegment &lineSeg);
    bool doLinesIntersect(LineSegment &, LineSegment &);
    LineSegment getLineSegment(VECTOR2D &, VECTOR2D &);
    double getDistance(VECTOR2D &p1, VECTOR2D &p2);
    double getDistanceSquared(VECTOR2D &p1, VECTOR2D &p2);
    VECTOR2D* getNearestPoint(VECTOR2D &p);
    int searchAPoint(VECTOR2D *p);
    double getGradient(VECTOR2D &, VECTOR2D &);
    double getMinimumDistancePointToLine(Geometry::LineSegment &l, VECTOR2D &p);
    double getMinimumDistancePointToLineSquared(Geometry::LineSegment &l, VECTOR2D &p);
    LineSegment * getNearestLine(VECTOR2D &p);
    LineSegment getMinimumDistanceLine(Geometry::LineSegment &l, VECTOR2D &p);
    LineSegment * getNextLineSegment(LineSegment* l);
    double correctAngle(double);
};

#endif // GEOMETRY_H


// ==========================================================================================


// //
// // Created by kadupitiya on 9/7/18.
// //

// #ifndef GEOMETRY_H
// #define GEOMETRY_H

// #include <geometry_msgs/Point.h>
// #include <geometry_msgs/Quaternion.h>
// #include "vector2D.h"

// using namespace std;
// using namespace geometry_msgs;

// class Geometry {

// public:

//     struct LineSegment {
//         VECTOR2D startP;
//         VECTOR2D endP;
//         double gradient;
//         double disatance;
//         double disatanceSquared;
//         double disatanceToAObj;
//         LineSegment *shortestDistLine;

//     };

//     vector<VECTOR2D> trajectory;
//     vector<LineSegment> path;

//     map<double, VECTOR2D*> distances;
//     map<double, LineSegment*> distancesPath;


//     //Member functions

//     bool checkInsideRectSimple(VECTOR2D &, VECTOR2D &, VECTOR2D &, VECTOR2D &, VECTOR2D &);

//     double crossProduct(VECTOR2D &, VECTOR2D &);

//     double dotProduct(VECTOR2D &, VECTOR2D &);

//     bool doBoundingBoxesIntersect(LineSegment &, LineSegment &);

//     bool isPointOnLine(LineSegment &, VECTOR2D &);

//     bool isPointRightOfLine(LineSegment &, VECTOR2D &);

//     bool lineSegmentTouchesOrCrossesLine(LineSegment &a, LineSegment &b);

//     LineSegment getBoundingBox(LineSegment &lineSeg);

//     bool doLinesIntersect(LineSegment &, LineSegment &);

//     LineSegment getLineSegment(VECTOR2D &, VECTOR2D &);

//     double getDistance(VECTOR2D &p1, VECTOR2D &p2);

//     double getDistanceSquared(VECTOR2D &p1, VECTOR2D &p2);

//     VECTOR2D* getNearestPoint(VECTOR2D &p);

//     int searchAPoint(VECTOR2D *p);

//     double getGradient(VECTOR2D &, VECTOR2D &);

//     double getMinimumDistancePointToLine(Geometry::LineSegment &l, VECTOR2D &p);

//     double getMinimumDistancePointToLineSquared(Geometry::LineSegment &l, VECTOR2D &p);

//     LineSegment * getNearestLine(VECTOR2D &p);

//     LineSegment getMinimumDistanceLine(Geometry::LineSegment &l, VECTOR2D &p);

//     LineSegment * getNextLineSegment(LineSegment* l);

//     double correctAngle(double);

// };


// #endif //GEOMETRY_H