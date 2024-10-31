//
// Created by wt on 2020/7/2.
//
 
#ifndef DEMO_SWEEPING_ROBOT_MAINWINDOW_H
#define DEMO_SWEEPING_ROBOT_MAINWINDOW_H
#include <QWidget>
#include <QLineEdit>
#include <QPushButton>
#include <QFormLayout>
#include <ros/ros.h>
#include <QLabel>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <std_msgs/Float64.h>
#include <thread>
 
class MainWindow: public QWidget {
private:
    QFormLayout layout;
    QLineEdit xEdit;
    QLineEdit yEdit;
    QLabel xLabel;
    QLabel yLabel;
    QLabel thetaLabel;
    QPushButton btn;
    QPushButton sweepBtn;
    QLineEdit kpEdit;
    QLineEdit kdEdit;
    QLineEdit kiEdit;
    //发布者
    ros::Publisher publisher;
    //plot发布者
    ros::Publisher plotPublisher;
    //订阅者
    ros::Subscriber subscriber;
    //保存的当前x和y
    float curx;
    float cury;
    //小乌龟的角度
    float curTheta;
public:
    MainWindow(ros::NodeHandle node,QWidget* parent = Q_NULLPTR);
 
    ~MainWindow();
    //点击
    void click();
    //扫地
    void sweep();
    //纵向扫地
    void vSweep();
    //求距离
    float distance(float curx,float cury,float dstx,float dsty);
    //回调
    void callBack(const turtlesim::Pose::ConstPtr & pose);
    //控制小乌龟
    void controlTurtle(float dstx,float dsty);
    //计算需要转的角度
    float caclAngle(float dstx,float dsty,float curx,float cury,float theta);
};
 
 
#endif //DEMO_SWEEPING_ROBOT_MAINWINDOW_H