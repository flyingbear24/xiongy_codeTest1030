// ROS集成开发环境搭建 //

# 1.创建《工作空间》并初始化：
    mkdir -p 自定义空间名称/src
    cd 自定义空间名称
    catkin_make

# 2.进入 src 创建《ros功能包》并添加依赖：
    cd src
    catkin_create_pkg 自定义ROS包名 roscpp rospy std_msgs

# 3.进入 ros 包的 src 目录编辑源文件：
    cd 自定义的包
    C++源码实现(文件名自定义)

# 4.编辑 ros 包下的 Cmakelist.txt文件：
    add_executable(步骤3的源文件名
    src/步骤3的源文件名.cpp
    )
    target_link_libraries(步骤3的源文件名
    ${catkin_LIBRARIES}
    )

# 5.进入工作空间目录并编译
    cd 自定义空间名称
    catkin_make

# 执行：
    先启动命令行1：
    roscore
    再启动命令行2：
    cd 工作空间
    source ./devel/setup.bash
    rosrun 包名 C++节点
*/



/*
使用 echo 命令将 source ~/工作空间/devel/setup.bash 添加到 .bashrc 文件的末尾是一种快速且方便的方法。下面是具体的步骤：
打开你的终端，输入以下命令：

    echo "source ~/工作空间/devel/setup.bash" >> ~/.bashrc

这个命令的作用是将 source ~/工作空间/devel/setup.bash 这一行文本追加到你的 .bashrc 文件的末尾。>> 是一个重定向操作符，它将命令左边的输出追加到右边指定的文件中。
完成这个操作后，每次你打开一个新的终端窗口或者新的终端会话时，.bashrc 文件中的内容都会被执行，包括你刚刚添加的 source 命令。这意味着 ~/工作空间/devel/setup.bash 脚本会在每个新的终端会话中自动执行。
如果你想要立即应用这个更改，而不是等待下一个终端会话，你可以在当前终端会话中执行以下命令：

    source ~/.bashrc

这将重新加载 .bashrc 文件，应用你刚刚所做的更改。
*/


# launch文件演示：

1.选定功能包右击 ---> 添加 launch 文件夹
2.选定 launch 文件夹右击 ---> 添加 launch 文件
3.编辑 launch 文件内容

运行 launch 文件:
    roslaunch 包名 launch文件名

// 这个命令可以直接杀死一个运行中的节点：
// rosnode kill node_name


