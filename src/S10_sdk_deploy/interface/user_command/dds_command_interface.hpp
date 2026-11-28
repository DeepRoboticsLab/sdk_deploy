/**
 * @file dds_command_interface.hpp
 * @brief DDS-based user command interface via ROS 2 topic subscription
 *        参考 basic_server 发出端，订阅 /STEER 和 /GAMEPAD_KEY 话题
 * @author DeepRobotics
 * @version 2.0
 * @date 2026-08-27
 *
 * @copyright Copyright (c) 2026  DeepRobotics
 *
 */
#pragma once

#include "user_command_interface.h"

#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

using namespace interface;
using namespace types;

namespace rclcpp {
class Node;
}

class DdsCommandInterface : public UserCommandInterface {
private:
    std::shared_ptr<rclcpp::Node> node_;
    struct RosImpl;
    std::unique_ptr<RosImpl> ros_;

    std::mutex cmd_mutex_;

    // 按键名 → KeyCode 映射（与 gamepad_interface.hpp 保持一致，兼容 G20/G12）
    static std::unordered_map<std::string, KeyCode> keyMap;

public:
    DdsCommandInterface(RobotName robot_name);

    ~DdsCommandInterface();

    /**
     * @brief 设置 ROS 2 节点（需在 Start 前调用）
     * @param node  共享的 rclcpp 节点指针
     */
    void SetNode(std::shared_ptr<rclcpp::Node> node) {
        node_ = node;
    }

    void Start() override;

    void Stop() override;

    UserCommand* GetUserCommand() override {
        return usr_cmd_;
    }
};
