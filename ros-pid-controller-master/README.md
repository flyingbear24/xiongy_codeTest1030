本案例用于在ROS2环境下修改PID参数验证不同PID设置值情况下变化

在编译前请进入到功能包下安装相关依赖
```bash
sudo chmod 777 install_prereq.sh
./install_prereq.sh
```

回到工作空间目录，编译功能包

```bash
catkin_make
```

编译后使用

```bash
export TURTLEBOT3_MODEL=burger
roslaunch tb3_control control.launch
```

## 运行时的节点关系

![](./assets/ros1-graph.png)
