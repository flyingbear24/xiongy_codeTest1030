/*

# ros::ok() 
    C++中可以通过 ros::ok() 来判断节点状态是否正常，导致节点退出的原因主要有如下几种:
    节点接收到了关闭信息，比如常用的 ctrl + c 快捷键就是关闭节点的信号；
    同名节点启动，导致现有节点退出；
    程序中的其他部分调用了节点关闭相关的API,C++中是ros::shutdown()


# 日志相关的函数也是极其常用的，在ROS中日志被划分成如下级别:
    DEBUG(调试):只在调试时使用，此类消息不会输出到控制台；
    INFO(信息):标准消息，一般用于说明系统内正在执行的操作；
    WARN(警告):提醒一些异常情况，但程序仍然可以执行；
    ERROR(错误):提示错误信息，此类错误会影响程序运行；
    FATAL(严重错误):此类错误将阻止节点继续运行。

# vector遍历for：
    #include <iostream>
    #include <vector>
    #include <string>
    int main() {
        // 创建一个包含字符串的 vector
        std::vector<std::string> param_status = {"active", "pending", "completed"};
        // 使用范围基于for循环遍历 vector
        for (const auto& status : param_status) {
            std::cout << status << std::endl;
        }
        return 0;
    }

在这个例子中，我们使用了C++11引入的范围基于for循环，它简化了代码，使得遍历容器更加直观。
在这个循环中，status 是 param_status 中每个元素的副本，我们使用 const auto& 来避免不必要的复制，并且保证不会修改元素。

注意，auto 关键字自动推断变量的类型，而 const 确保我们不会意外地修改容器中的元素。
如果你确定不会修改元素，并且想要提高效率，可以使用 const auto&。如果你需要修改元素，可以去掉 const，使用 auto&。

*/