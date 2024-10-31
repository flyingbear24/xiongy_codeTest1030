//
// Created by wt on 2020/7/2.
//
#include <iostream>
#include <ros/ros.h>
#include <QApplication>
#include "test01_pid/MainWindow.h"
using namespace std;
 
int main(int argc,char *argv[]){
    //节点名
    string nodeName = "sweeping_node";
    //初始化节点
    ros::init(argc,argv,nodeName);
    //创建节点
    ros::NodeHandle node;
    /*-------------------------- 异步spiner --------------------------*/
    ros::AsyncSpinner spinner(1);
    spinner.start();
    /*-------------------------- qt --------------------------*/
    QApplication app(argc,argv);
    MainWindow w(node);
    w.show();
    return QApplication::exec();
}