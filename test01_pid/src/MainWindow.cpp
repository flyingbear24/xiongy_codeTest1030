//
// Created by wt on 2020/7/2.
//
 
#include "test01_pid/MainWindow.h"
 
MainWindow::MainWindow(ros::NodeHandle node, QWidget *parent) : QWidget(parent),
btn("执行"),sweepBtn("扫地") {
    //发布者
    publisher = node.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 5);
    //订阅者
    subscriber = node.subscribe("/turtle1/pose", 5, &MainWindow::callBack, this);
    //plot发布者
    plotPublisher = node.advertise<std_msgs::Float64>("/tutle/speed", 5);
 
    setLayout(&layout);
    //默认值
    xEdit.setText("5.544444561");
    yEdit.setText("5.544444561");
    layout.addRow("目标x:", &xEdit);
    layout.addRow("目标y:", &yEdit);
    layout.addRow("当前x:", &xLabel);
    layout.addRow("当前y:", &yLabel);
    layout.addRow("当前theta:", &thetaLabel);
    kpEdit.setText("0.1");
    kdEdit.setText("0.1");
    kiEdit.setText("0.1");
    layout.addRow("kp:",&kpEdit);
    layout.addRow("kd:",&kdEdit);
    layout.addRow("ki:",&kiEdit);
    layout.addRow("", &btn);
    layout.addRow("", &sweepBtn);
 
    //信号和槽绑定
    connect(&btn, &QPushButton::clicked, this, &MainWindow::click);
    connect(&sweepBtn, &QPushButton::clicked, this, &MainWindow::sweep);
}
 
MainWindow::~MainWindow() {
 
}
 
//真实的速度
void MainWindow::click() {
    //当前的x和y
    //目标的x和y
    float dstx = xEdit.text().toFloat();
    float dsty = yEdit.text().toFloat();
    //开启线程执行耗时操作
    new std::thread(&MainWindow::controlTurtle, this, dstx, dsty);
}
 
//转向
void MainWindow::controlTurtle(float dstx, float dsty) {
    //走的距离
    float dst = distance(curx, cury, dstx, dsty);
    //走的时间
    float time = 5.0;
    //调头时间
    float turnTime = time/12;
    //频率
    float hz = 10;
    //频率
    ros::Rate rate(hz);
    //当前速度
    float curSpeed = 0;
    //记录上一次误差
    float lastError = 0;
    //总误差
    float totalError = 0;
    //kp系数
    float kp = kpEdit.text().toFloat();
    //kd系数
    float kd = kdEdit.text().toFloat();
    //ki系数
    float ki = kiEdit.text().toFloat();
 
    geometry_msgs::Twist data;
    while (distance(curx, cury, dstx, dsty) > 0.1) {
        //目标速度
        float tarSpeed = distance(curx, cury, dstx, dsty) / time;
        /*-------------------------- p --------------------------*/
        //误差= 目标速度 - 当前速度
        float pError = tarSpeed - curSpeed;
        //调节速度  当前速度=当前速度 + 误差*kp系数(比例系数)
        curSpeed += pError * kp;
        /*-------------------------- d --------------------------*/
        // dError =  当前误差-记录的上一次误差
        float dError = pError - lastError;
        //   
        curSpeed += dError * kd;
        //记录当前误差
        lastError = pError;
        /*-------------------------- i --------------------------*/
        // 总误差 = 当前误差累加
        totalError += pError;
        curSpeed+=totalError*ki;
 
        //数据
        data.linear.x = curSpeed;
        //角速度
        data.angular.z = caclAngle(dstx,dsty,curx,cury,curTheta)/turnTime;
        //移动 (只会按照当前这个速度走1秒钟)
        publisher.publish(data);
 
        //数据
        std_msgs::Float64 sp;
        sp.data = curSpeed;
        //发送速度给plot展示
        plotPublisher.publish(sp);
        //剩下的时间
        time -= 1 / hz;
        //睡眠
        rate.sleep();
    }
    //把速度设置为0
    //数据
    data.linear.x = 0;
    data.angular.z = 0;
    //移动 (只会按照当前这个速度走1秒钟)
    publisher.publish(data);
}
 
//D算法
//void MainWindow::controlTurtle(float dstx, float dsty) {
//    //走的距离
//    float dst = distance(curx, cury, dstx, dsty);
//    //走的时间
//    float time = 5.0;
//    //频率
//    float hz = 10;
//    //频率
//    ros::Rate rate(hz);
//    //当前速度
//    float curSpeed = 0;
//    //记录上一次误差
//    float lastError = 0;
//    //总误差
//    float totalError = 0;
//    //kp系数
//    float kp = kpEdit.text().toFloat();
//    //kd系数
//    float kd = kdEdit.text().toFloat();
//    //ki系数
//    float ki = kiEdit.text().toFloat();
//
//    geometry_msgs::Twist data;
//    while (distance(curx, cury, dstx, dsty) > 0.1) {
//        //目标速度
//        float tarSpeed = distance(curx, cury, dstx, dsty) / time;
//        /*-------------------------- p --------------------------*/
//        //误差
//        float pError = tarSpeed - curSpeed;
//        //调节速度
//        curSpeed += pError * kp;
//        /*-------------------------- d --------------------------*/
//        float dError = pError - lastError;
//        curSpeed += dError * kd;
//        lastError = pError;
//        /*-------------------------- i --------------------------*/
//        totalError += pError;
//        curSpeed+=totalError*ki;
//
//        //数据
//        data.linear.x = curSpeed;
//        data.angular.z = 0;
//        //移动 (只会按照当前这个速度走1秒钟)
//        publisher.publish(data);
//
//        //数据
//        std_msgs::Float64 sp;
//        sp.data = curSpeed;
//        //发送速度给plot展示
//        plotPublisher.publish(sp);
//        //剩下的时间
//        time -= 1 / hz;
//        //睡眠
//        rate.sleep();
//    }
//    //把速度设置为0
//    //数据
//    data.linear.x = 0;
//    data.angular.z = 0;
//    //移动 (只会按照当前这个速度走1秒钟)
//    publisher.publish(data);
//}
 
 
//P算法
//void MainWindow::controlTurtle(float dstx, float dsty) {
//    //走的距离
//    float dst = distance(curx,cury,dstx,dsty);
//    //走的时间
//    float time = 5.0;
//    //频率
//    float hz = 10;
//    //频率
//    ros::Rate rate(hz);
//    //当前速度
//    float curSpeed = 0;
//    //kp系数
//    float kp = 0.1;
//
//    geometry_msgs::Twist data;
//    while (distance(curx,cury,dstx,dsty)>0.1){
//        //目标速度
//        float tarSpeed = distance(curx,cury,dstx,dsty)/time;
//        //误差
//        float pError = tarSpeed-curSpeed;
//        //调节速度
//        curSpeed += pError*kp;
//
//        //数据
//        data.linear.x = curSpeed;
//        data.angular.z = 0;
//        //移动 (只会按照当前这个速度走1秒钟)
//        publisher.publish(data);
//
//        //数据
//        std_msgs::Float64 sp;
//        sp.data = curSpeed;
//        //发送速度给plot展示
//        plotPublisher.publish(sp);
//        //剩下的时间
//        time -= 1/hz;
//        //睡眠
//        rate.sleep();
//    }
//    //把速度设置为0
//    //数据
//    data.linear.x = 0;
//    data.angular.z = 0;
//    //移动 (只会按照当前这个速度走1秒钟)
//    publisher.publish(data);
//}
 
//计算速度
//void MainWindow::controlTurtle(float dstx, float dsty) {
//    //走的距离
//    float dst = distance(curx,cury,dstx,dsty);
//    //走的时间
//    float time = 5.0;
//    //频率
//    float hz = 10;
//    //频率
//    ros::Rate rate(hz);
//    //速度
//    float speed = dst/time;
//    geometry_msgs::Twist data;
//    while (distance(curx,cury,dstx,dsty)>0.1){
//        speed = distance(curx,cury,dstx,dsty)/time;
//        //数据
//        data.linear.x = speed;
//        data.angular.z = 0;
//        //移动 (只会按照当前这个速度走1秒钟)
//        publisher.publish(data);
//
//        //数据
//        std_msgs::Float64 sp;
//        sp.data = speed;
//        //发送速度给plot展示
//        plotPublisher.publish(sp);
//        //剩下的时间
//        time -= 1/hz;
//        //睡眠
//        rate.sleep();
//    }
//    //把速度设置为0
//    //数据
//    data.linear.x = 0;
//    data.angular.z = 0;
//    //移动 (只会按照当前这个速度走1秒钟)
//    publisher.publish(data);
//}
 
//指定时间走完
//void MainWindow::click() {
//    //当前的x和y
//    //目标的x和y
//    float dstx = xEdit.text().toFloat();
//    float dsty = yEdit.text().toFloat();
//    //走的距离
//    float dst = distance(curx,cury,dstx,dsty);
//    //走的时间
//    float time = 5.0;
//    //频率
//    float hz = 10;
//    //频率
//    ros::Rate rate(hz);
//    //速度
//    float speed = dst/time;
//    geometry_msgs::Twist data;
//    while (distance(curx,cury,dstx,dsty)>0.1){
//        //数据
//        data.linear.x = speed;
//        data.angular.z = 0;
//        //移动 (只会按照当前这个速度走1秒钟)
//        publisher.publish(data);
//
//        //数据
//        std_msgs::Float64 sp;
//        sp.data = speed;
//        //发送速度给plot展示
//        plotPublisher.publish(sp);
//        //睡眠
//        rate.sleep();
//    }
//    //把速度设置为0
//    //数据
//    data.linear.x = 0;
//    data.angular.z = 0;
//    //移动 (只会按照当前这个速度走1秒钟)
//    publisher.publish(data);
//}
 
//指定时间走完
//void MainWindow::click() {
//    //当前的x和y
//    //目标的x和y
//    float dstx = xEdit.text().toFloat();
//    float dsty = yEdit.text().toFloat();
//    //走的距离
//    float dst = distance(curx,cury,dstx,dsty);
//    //走的时间
//    float time = 5.0;
//    //频率
//    float hz = 10;
//    //频率
//    ros::Rate rate(hz);
//    //速度
//    float speed = dst/time;
//    //走的距离
//    float runDst = 0;
//    geometry_msgs::Twist data;
//    while (runDst<dst){
//        //数据
//        data.linear.x = speed;
//        data.angular.z = 0;
//        //移动 (只会按照当前这个速度走1秒钟)
//        publisher.publish(data);
//        //增加走的距离
//        runDst += speed/hz;
//        //睡眠
//        rate.sleep();
//    }
//    //把速度设置为0
//    //数据
//    data.linear.x = 0;
//    data.angular.z = 0;
//    //移动 (只会按照当前这个速度走1秒钟)
//    publisher.publish(data);
//}
 
//指定时间走完
//void MainWindow::click() {
//    //当前的x和y
//    //目标的x和y
//    float dstx = xEdit.text().toFloat();
//    float dsty = yEdit.text().toFloat();
//    //走的距离
//    float dst = distance(curx,cury,dstx,dsty);
//    //走的时间
//    float time = 5.0;
//    //频率
//    float hz = 10;
//    //频率
//    ros::Rate rate(hz);
//    //速度
//    float speed = dst/time;
//    geometry_msgs::Twist data;
//    for (int i = 0; i < (int)(time*hz); ++i) {
//        //数据
//        data.linear.x = speed;
//        data.angular.z = 0;
//        //移动 (只会按照当前这个速度走1秒钟)
//        publisher.publish(data);
//        //睡眠
//        rate.sleep();
//    }
//    //把速度设置为0
//    //数据
//    data.linear.x = 0;
//    data.angular.z = 0;
//    //移动 (只会按照当前这个速度走1秒钟)
//    publisher.publish(data);
//}
 
//5秒钟走完(问题)
//void MainWindow::click() {
//    //当前的x和y
//    //目标的x和y
//    float dstx = xEdit.text().toFloat();
//    float dsty = yEdit.text().toFloat();
//    //走的距离
//    float dst = distance(curx,cury,dstx,dsty);
//    //走的时间
//    float time = 5;
//    //频率
//    ros::Rate rate(1);
//    //速度
//    float speed = dst/time;
//    for (int i = 0; i < (int)time; ++i) {
//        //数据
//        geometry_msgs::Twist data;
//        data.linear.x = speed;
//        data.angular.z = 0;
//        //移动 (只会按照当前这个速度走1秒钟)
//        publisher.publish(data);
//        //睡眠
//        rate.sleep();
//    }
//}
 
//1秒钟走完
//void MainWindow::click() {
//    //当前的x和y
//    //目标的x和y
//    float dstx = xEdit.text().toFloat();
//    float dsty = yEdit.text().toFloat();
//    //走的距离
//    float dst = distance(curx,cury,dstx,dsty);
//    //走的时间
//    float time = 1;
//    //速度
//    float speed = dst/time;
//    //数据
//    geometry_msgs::Twist data;
//    data.linear.x = speed;
//    data.angular.z = 0;
//    //移动
//    publisher.publish(data);
//}
 
float MainWindow::distance(float curx, float cury, float dstx, float dsty) {
    return sqrt(pow(curx - dstx, 2) + pow(cury - dsty, 2));
}
 
void MainWindow::callBack(const turtlesim::Pose_<std::allocator<void>>::ConstPtr &pose) {
    curx = pose->x;
    cury = pose->y;
    curTheta = pose->theta;
    xLabel.setText(QString::number(curx));
    yLabel.setText(QString::number(cury));
    thetaLabel.setText(QString::number(pose->theta));
}
/**
 * 计算需要转的角度
 * @param dstx
 * @param dsty
 * @param curx
 * @param cury
 * @param theta 小乌龟当前角度
 * @return
 */
float MainWindow::caclAngle(float dstx,float dsty,float curx,float cury,float theta){
    float angle = atan2(dsty-cury,dstx-curx)-theta;
    //角度范围在0-180 -180-0
    if(angle>M_PI){
        angle -= 2*M_PI;
    } else if (angle<-M_PI){
        angle += 2*M_PI;
    }
    return angle;
}
 
void MainWindow::sweep() {
    new std::thread(&MainWindow::vSweep,this);
}
 
void MainWindow::vSweep() {
    for (int i = 1; i < 11; ++i) {
        controlTurtle((float)i,1);
        controlTurtle((float)i,10);
    }
}
 
 
 
 
 