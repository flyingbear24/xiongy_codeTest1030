/*

实现
# 1.准备
    先要保证不同计算机处于同一网络中，最好分别设置固定IP，如果为虚拟机，需要将网络适配器改为桥接模式；

# 2.配置文件修改
    分别修改不同计算机的 /etc/hosts 文件，在该文件中加入对方的IP地址和计算机名:
    主机端:
    从机的IP    从机计算机名
    从机端:
    主机的IP    主机计算机名

    设置完毕，可以通过 ping 命令测试网络通信是否正常。
        IP地址查看名: ifconfig
        计算机名称查看: hostname

# 3.配置主机IP
    配置主机的 IP 地址
    ~/.bashrc 追加
    export ROS_MASTER_URI=http://主机IP:11311
    export ROS_HOSTNAME=主机IP

# 4.配置从机IP
    配置从机的 IP 地址，从机可以有多台，每台都做如下设置:
    ~/.bashrc 追加
    export ROS_MASTER_URI=http://主机IP:11311
    export ROS_HOSTNAME=从机IP



*/