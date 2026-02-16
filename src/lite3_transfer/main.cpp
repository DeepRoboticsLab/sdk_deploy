#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "interface/robot/udp_interface.hpp"  // HardwareInterface 的声明
#include "interface/gemepad_command/retroid_gamepad_interface.hpp"
#include <string>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // 创建硬件接口节点
    auto hardware_node = std::make_shared<HardwareInterface>("lite3");

    // 启动 UDP 接收 & 切到 SDK 模式
    hardware_node->Start();

    // 从命令行参数获取手柄端口，默认为 12121
    int gamepad_port = 12121;
    // if (argc > 1) {
    //     gamepad_port = std::stoi(argv[1]);
    // }
    
    // 创建手柄发布者节点
    auto gamepad_pub = std::make_shared<RetroidGamepadPublisher>(gamepad_port);
    gamepad_pub->Start();


    // 使用多线程执行器同时运行两个节点
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(hardware_node);
    executor.add_node(gamepad_pub);
    
    // 进入 ROS2 事件循环：
    // - 定时器回调会以 200Hz 发布 /JOINTS_DATA 和 /IMU_DATA
    // - /JOINTS_CMD 订阅回调会把 drdds 命令转成 UDP 下发
    // - 手柄数据会发布到 /GAMEPAD_DATA
    executor.spin();

    // 退出时停止节点
    gamepad_pub->Stop();
    hardware_node->Stop();

    rclcpp::shutdown();
    return 0;
}