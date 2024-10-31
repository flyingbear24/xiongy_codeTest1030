#include "ros/ros.h"  // 包含ROS核心库
#include <tf/transform_listener.h>  // 包含tf库，用于坐标变换监听
#include <nav_msgs/Odometry.h>  // 包含里程计消息类型
#include <visualization_msgs/Marker.h>  // 包含可视化标记消息类型
#include <std_srvs/Empty.h>  // 包含标准服务类型
#include "geometry.h"  // 包含自定义的几何计算头文件
#include "pid.h"  // 包含自定义的PID控制器头文件

// 全局变量定义
tf::Point Odom_pos;  // 用于存储机器人当前位置
double Odom_yaw, Odom_v, Odom_w;  // 用于存储机器人当前朝向、线速度和角速度
ros::Publisher cmd_vel_pub, marker_pub;  // ROS发布者，用于发布速度命令和可视化标记
ros::Subscriber odom_sub;  // ROS订阅者，用于订阅里程计数据
int num_slice = 50;  // 轨迹切片数量
double maxSpeed = 2, distanceConst = 0.5;  // 最大速度和距离常数

// PID控制器参数
double dt = 0.1, maxT = M_PI, minT = -M_PI, Kp = 1, Ki = 0.02, Kd = 0.1;  // 角度PID参数
double dtS = 0.1, maxS = maxSpeed, minS = 0.0, KpS = 0.1, KiS = 0.02, KdS = 0.1;  // 速度PID参数

// 里程计数据的回调函数
void odomCallback(const nav_msgs::Odometry odom_msg) {
    tf::pointMsgToTF(odom_msg.pose.pose.position, Odom_pos);  // 将位置消息转换为tf::Point类型
    Odom_yaw = tf::getYaw(odom_msg.pose.pose.orientation);  // 获取机器人的朝向
    Odom_v = odom_msg.twist.twist.linear.x;  // 获取机器人的线速度
    Odom_w = odom_msg.twist.twist.angular.z;  // 获取机器人的角速度
}

// 在RViz中显示轨迹的函数
void displayLane(bool isTrajectoryPushed, Geometry &geometry) {
    static visualization_msgs::Marker path;  // 定义一个静态的可视化标记
    path.type = visualization_msgs::Marker::LINE_STRIP;  // 设置标记类型为线段
    path.header.frame_id = "odom";  // 设置坐标系为里程计坐标系
    path.header.stamp = ros::Time::now();  // 设置时间戳
    path.ns = "odom";  // 设置命名空间
    path.id = 0;  // 设置ID
    path.action = visualization_msgs::Marker::ADD;  // 设置操作为添加
    path.lifetime = ros::Duration();  // 设置生命周期
    path.color.b = 1.0;  // 设置颜色为蓝色
    path.color.a = 1.0;  // 设置颜色的透明度
    path.scale.x = 0.02;  // 设置线段的宽度
    path.pose.orientation.w = 1.0;  // 设置姿态的四元数

    static int slice_index = 0;  // 用于记录轨迹切片的索引

    VECTOR2D *prev = nullptr, *current = nullptr;  // 用于存储轨迹点的指针

    // 根据轨迹切片数量生成轨迹线
    while (path.points.size() <= num_slice) {       // 每次调用，path.points.size()重置为0
        geometry_msgs::Point p;
        float angle = slice_index * 2 * M_PI / num_slice;  // 计算角度
        slice_index++;
        p.x = 4 * cos(angle) - 1.0;  // 计算x坐标
        p.y = 4 * sin(angle) + 1.0;  // 计算y坐标
        p.z = 0;  // z坐标设置为0

        path.points.push_back(p);  // 将点添加到轨迹线

        if (!isTrajectoryPushed) {  // 如果轨迹尚未推送
            VECTOR2D *temp = new VECTOR2D(p.x, p.y);  // 创建一个新的向量
            geometry.trajectory.push_back(*temp);  // 将向量添加到轨迹

            current = temp;  // 更新当前点

            if (prev != nullptr) {  // 如果前一个点存在
                geometry.path.push_back(geometry.getLineSegment(*prev, *current));  // 添加线段
            }
            prev = current;  // 更新前一个点
        }
    }

    if (prev != nullptr && current != nullptr && current != prev)  // 如果前一个点和当前点都存在，且不相同
        geometry.path.push_back(geometry.getLineSegment(*prev, *current));  // 添加线段

    marker_pub.publish(path);  // 发布轨迹线
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "control");  // 初始化ROS节点
    ros::NodeHandle n("~");  // 创建节点句柄
    tf::TransformListener m_listener;  // 创建tf监听器
    tf::StampedTransform transform;  // 创建tf变换对象
    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);  // 广告速度命令话题
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);  // 广告可视化标记话题
    odom_sub = n.subscribe("odom", 10, odomCallback);  // 订阅里程计数据话题

    ros::Rate loop_rate(10);  // 设置循环频率为10Hz

    geometry_msgs::Twist tw_msg;  // 定义速度命令消息
    Geometry geometry;  // 创建几何计算对象

    int frame_count = 0;  // 用于记录循环次数
    PID pidTheta(dt, maxT, minT, Kp, Kd, Ki);  // 创建角度PID控制器对象
    PID pidVelocity(dtS, maxS, minS, KpS, KdS, KiS);  // 创建速度PID控制器对象
    
    double omega = 0.0;  // 用于存储角速度
    while (ros::ok()) {  // 当ROS节点处于OK状态时循环
        if (frame_count == 0)  // 如果是第一次循环
            displayLane(false, geometry);  // 显示轨迹
        else
            displayLane(true, geometry);  // 更新轨迹

        double speed = maxSpeed;  // 设置速度为最大速度
        VECTOR2D current_pos;  // 定义当前位置向量
        current_pos.x = Odom_pos.x();  // 设置当前位置的x坐标
        current_pos.y = Odom_pos.y();  // 设置当前位置的y坐标

        Geometry::LineSegment *lineSegment = geometry.getNearestLine(current_pos);  // 获取最近的线段
        Geometry::LineSegment lineSegmentPerpen = geometry.getMinimumDistanceLine(*lineSegment, current_pos);  // 获取垂直于当前位置的线段
        Geometry::LineSegment *nextLinesegment = geometry.getNextLineSegment(lineSegment);  // 获取下一条线段

        double targetDistanceWithEnd = geometry.getDistanceSquared(current_pos, lineSegment->endP);  // 计算到线段终点的距离平方
        double targetDistanceWithStart = geometry.getDistanceSquared(current_pos, lineSegment->startP);  // 计算到线段起点的距离平方
        double targetAngleWithPerpen = geometry.getGradient(current_pos, lineSegment->endP);  // 计算到线段终点的梯度

        double angleError = 0.0;  // 初始化角度误差

        VECTOR2D target = lineSegment->endP;  // 设置目标点为线段终点
        double targetAngle = geometry.getGradient(current_pos, target);  // 计算到目标点的梯度
        double distanceToClosestPath = abs(lineSegment->disatanceToAObj);  // 计算到最近路径的距离
        double targetAnglePerpen = geometry.getGradient(current_pos, lineSegmentPerpen.endP);  // 计算到垂直线段终点的梯度

        if (distanceToClosestPath < distanceConst) {  // 如果距离小于常数
            double directional = targetAngle;  // 设置方向为目标点的梯度

            double discripancy = targetAnglePerpen - directional;  // 计算误差
            discripancy = geometry.correctAngle(discripancy);  // 校正误差

            discripancy = 0.5 * discripancy / distanceConst * abs(distanceToClosestPath);  // 计算修正后的误差

            double combined = targetAngle + discripancy;  //
        angleError = combined - Odom_yaw;  // 计算角度误差
    } else {
        angleError = targetAnglePerpen - Odom_yaw;  // 计算角度误差
    }
    if (targetDistanceWithEnd < 0.5) {  // 如果距离终点的距离小于0.5
        double futureAngleChange = nextLinesegment->gradient - lineSegment->gradient;  // 计算未来的角度变化
        futureAngleChange = geometry.correctAngle(futureAngleChange);  // 校正角度变化
        futureAngleChange = futureAngleChange / distanceConst * abs(targetDistanceWithEnd);;  // 计算修正后的角度变化
        double combined = targetAngle + futureAngleChange;  // 计算修正后的目标点梯度
        angleError = combined - Odom_yaw;  // 计算角度误差
    } else {
        angleError = geometry.getGradient(current_pos, lineSegmentPerpen.endP) - Odom_yaw;  // 计算角度误差
    }

    double speedError = 0.0;  // 初始化速度误差

    if (targetDistanceWithStart < 0.7 || targetDistanceWithEnd < 0.7) {  // 如果距离起点或终点的距离小于0.7
        double targetDistance = std::min(targetDistanceWithStart, targetDistanceWithEnd);  // 计算最小距离
        speedError = 0.3 * maxSpeed * exp(-abs(targetDistance));  // 计算速度误差
        speed = pidVelocity.calculate(maxSpeed, -speedError);  // 计算速度
    }

    angleError = geometry.correctAngle(angleError);  // 校正角度误差
    double omega = pidTheta.calculate(0, -angleError);  // 计算角速度

    ROS_INFO("Odom_yaw %f, Angle Error: %f , omega: %f Speed %f", Odom_yaw, angleError, omega, Odom_v);  // 打印信息

    tw_msg.linear.x = speed;  // 设置线速度---calculate计算的PID输出---
    tw_msg.angular.z = omega;  // 设置角速度---calculate计算的PID输出---
    cmd_vel_pub.publish(tw_msg);  // 发布速度命令

    frame_count++;  // 更新循环次数

    ros::spinOnce();  // 处理回调函数
    loop_rate.sleep();  // 等待下一个循环
}

return 0;  // 返回0

}


// =======================================================================================


// #include "ros/ros.h"
// #include <tf/transform_listener.h>
// #include <nav_msgs/Odometry.h>
// #include <visualization_msgs/Marker.h>
// #include <std_srvs/Empty.h>
// #include "geometry.h"
// #include "pid.h"

// tf::Point Odom_pos;
// double Odom_yaw, Odom_v, Odom_w;

// ros::Publisher cmd_vel_pub, marker_pub;
// ros::Subscriber odom_sub;
// int num_slice = 50;
// double maxSpeed = 0.5, distanceConst = 0.5;
// // double dt = 0.1, maxT = M_PI, minT = -M_PI, Kp = 0.3, Ki = 0.05, Kd = 0.01;
// double dt = 0.1, maxT = M_PI, minT = -M_PI, Kp = 1, Ki = 0.02, Kd = 0.1;
// // double dtS = 0.1, maxS = maxSpeed, minS = 0.0, KpS = 0.08, KiS = 0.01, KdS = 0.005;
// double dtS = 0.1, maxS = maxSpeed, minS = 0.0, KpS = 0.1, KiS = 0.02, KdS = 0.1;


// void odomCallback(const nav_msgs::Odometry odom_msg) {
//     tf::pointMsgToTF(odom_msg.pose.pose.position, Odom_pos);
//     Odom_yaw = tf::getYaw(odom_msg.pose.pose.orientation);
//     Odom_v = odom_msg.twist.twist.linear.x;
//     Odom_w = odom_msg.twist.twist.angular.z;
// }

// void displayLane(bool isTrajectoryPushed, Geometry &geometry) {
//     static visualization_msgs::Marker path;
//     path.type = visualization_msgs::Marker::LINE_STRIP;
//     path.header.frame_id = "odom";
//     path.header.stamp = ros::Time::now();
//     path.ns = "odom";
//     path.id = 0;
//     path.action = visualization_msgs::Marker::ADD;
//     path.lifetime = ros::Duration();
//     path.color.b = 1.0;
//     path.color.a = 1.0;
//     path.scale.x = 0.02;
//     path.pose.orientation.w = 1.0;

//     static int slice_index = 0;

//     VECTOR2D *prev = nullptr, *current = nullptr;

//     while (path.points.size() <= num_slice) {
//         geometry_msgs::Point p;
//         float angle = slice_index * 2 * M_PI / num_slice;
//         slice_index++;
//         p.x = 4 * cos(angle) - 0.5;
//         p.y = 4 * sin(angle) + 1.0;
//         p.z = 0;

//         path.points.push_back(p);

//         if (!isTrajectoryPushed) {
//             VECTOR2D *temp = new VECTOR2D(p.x, p.y);
//             geometry.trajectory.push_back(*temp);
//             current = temp;

//             if (prev != nullptr) {
//                 geometry.path.push_back(geometry.getLineSegment(*prev, *current));
//             }
//             prev = current;
//         }
//     }

//     if (prev != nullptr && current != nullptr && current != prev)
//         geometry.path.push_back(geometry.getLineSegment(*prev, *current));

//     marker_pub.publish(path);
// }

// int main(int argc, char **argv) {
//     ros::init(argc, argv, "control");
//     ros::NodeHandle n("~");
//     tf::TransformListener m_listener;
//     tf::StampedTransform transform;
//     cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
//     marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
//     odom_sub = n.subscribe("odom", 10, odomCallback);
//     ros::Rate loop_rate(10);

//     geometry_msgs::Twist tw_msg;
//     Geometry geometry;

//     int frame_count = 0;
//     PID pidTheta(dt, maxT, minT, Kp, Kd, Ki);
//     PID pidVelocity(dtS, maxS, minS, KpS, KdS, KiS);
    
//     double omega = 0.0;
//     while (ros::ok()) {
//         if (frame_count == 0)
//             displayLane(false, geometry);
//         else
//             displayLane(true, geometry);

//         double speed = maxSpeed;
//         VECTOR2D current_pos;
//         current_pos.x = Odom_pos.x();
//         current_pos.y = Odom_pos.y();

//         Geometry::LineSegment *lineSegment = geometry.getNearestLine(current_pos);
//         Geometry::LineSegment lineSegmentPerpen = geometry.getMinimumDistanceLine(*lineSegment, current_pos);
//         Geometry::LineSegment *nextLinesegment = geometry.getNextLineSegment(lineSegment);

//         double targetDistanceWithEnd = geometry.getDistanceSquared(current_pos, lineSegment->endP);
//         double targetDistanceWithStart = geometry.getDistanceSquared(current_pos, lineSegment->startP);
//         double targetAngleWithPerpen = geometry.getGradient(current_pos, lineSegment->endP);

//         double angleError = 0.0;

//         VECTOR2D target = lineSegment->endP;
//         double targetAngle = geometry.getGradient(current_pos, target);
//         double distanceToClosestPath = abs(lineSegment->disatanceToAObj);
// double targetAnglePerpen = geometry.getGradient(current_pos, lineSegmentPerpen.endP);
//         if (distanceToClosestPath < distanceConst) {

//             double directional = targetAngle;

//             double discripancy = targetAnglePerpen - directional;
//             discripancy = geometry.correctAngle(discripancy);

//             discripancy = 0.5* discripancy / distanceConst * abs(distanceToClosestPath);

//             double combined = targetAngle + discripancy;

//             angleError = combined - Odom_yaw;

//         } else {
//             angleError = targetAnglePerpen - Odom_yaw;
//         }
//         if (targetDistanceWithEnd < 0.5) {
//             double futureAngleChange = nextLinesegment->gradient - lineSegment->gradient;
//             futureAngleChange = geometry.correctAngle(futureAngleChange);
//             futureAngleChange = futureAngleChange / distanceConst * abs(targetDistanceWithEnd);;
//             double combined = targetAngle + futureAngleChange;
//             angleError = combined - Odom_yaw;
//         } else {
//             angleError = geometry.getGradient(current_pos, lineSegmentPerpen.endP) - Odom_yaw;
//         }

//         double speedError = 0.0;

//         if (targetDistanceWithStart < 0.7 || targetDistanceWithEnd < 0.7) {
//             double targetDistance = std::min(targetDistanceWithStart, targetDistanceWithEnd);
//             speedError = 0.3 * maxSpeed * exp(-abs(targetDistance));
//             speed = pidVelocity.calculate(maxSpeed, -speedError);
//         }

//         angleError = geometry.correctAngle(angleError);
//         double omega = pidTheta.calculate(0, -angleError);

//         ROS_INFO("Odom_yaw %f, Angle Error: %f , omega: %f Speed %f", Odom_yaw, angleError, omega, Odom_v);

//         tw_msg.linear.x = speed;
//         tw_msg.angular.z = omega;
//         cmd_vel_pub.publish(tw_msg);

//         frame_count++;
//         ros::spinOnce();
//         loop_rate.sleep();
//     }

//     return 0;
// }

