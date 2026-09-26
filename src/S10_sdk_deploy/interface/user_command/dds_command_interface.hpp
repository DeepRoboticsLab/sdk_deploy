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
#include "drdds/msg/steer.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cstring>
#include <iostream>
#include <mutex>
#include <string>
#include <unordered_map>

using namespace interface;
using namespace types;

class DdsCommandInterface : public UserCommandInterface {
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<drdds::msg::Steer>::SharedPtr steer_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr key_sub_;

    std::mutex cmd_mutex_;

    // 按键名 → KeyCode 映射（与 gamepad_interface.hpp 保持一致，兼容 G20/G12）
    static std::unordered_map<std::string, KeyCode> keyMap;

    void SteerCallback(const drdds::msg::Steer::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        // 轴指令映射：参考 basic_server cmdHandle_Steer
        //   x → 前后(forward), y → 左右(side), yaw → 偏航(turnning)
        usr_cmd_->forward_vel_scale = msg->data.x;
        usr_cmd_->side_vel_scale = msg->data.y;
        usr_cmd_->turnning_vel_scale = msg->data.yaw;

        // 周期性打印轴指令（降频）
        static auto last_print = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration<double>(now - last_print).count() >= 0.5) {
            last_print = now;
            std::cout << "[DDS] Steer X=" << usr_cmd_->forward_vel_scale
                      << " Y=" << usr_cmd_->side_vel_scale
                      << " Yaw=" << usr_cmd_->turnning_vel_scale << std::endl;
        }
    }

    void KeyCallback(const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        const std::string& keyStr = msg->data;
        std::cout << "[DDS] Key: " << keyStr << std::endl;

        if (keyMap.count(keyStr)) {
            KeyCode code = keyMap[keyStr];
            switch (code) {
                case KeyCode::L1:
                    if (msfb_->GetCurrentState() == RobotMotionState::WaitingForStand
                        || msfb_->GetCurrentState() == RobotMotionState::LieDown) {
                        usr_cmd_->target_mode = uint8_t(RobotMotionState::StandingUp);
                        std::cout << "[DDS][MODE] Standing Up" << std::endl;
                    }
                    break;
                case KeyCode::L2:
                    if (msfb_->GetCurrentState() == RobotMotionState::StandingUp) {
                        usr_cmd_->target_mode = uint8_t(RobotMotionState::RLControlMode);
                        std::cout << "[DDS][MODE] RL Control" << std::endl;
                    }
                    break;
                case KeyCode::R1:
                    if (msfb_->GetCurrentState() == RobotMotionState::StandingUp
                        || msfb_->GetCurrentState() == RobotMotionState::RLControlMode) {
                        usr_cmd_->target_mode = uint8_t(RobotMotionState::LieDown);
                        std::cout << "[DDS][MODE] Lie Down" << std::endl;
                    }
                    break;
                case KeyCode::R2:
                    usr_cmd_->target_mode = uint8_t(RobotMotionState::JointDamping);
                    std::cout << "[DDS][MODE] Joint Damping" << std::endl;
                    break;
                default:
                    break;
            }
        }
    }

public:
    DdsCommandInterface(RobotName robot_name) : UserCommandInterface(robot_name) {
        std::memset(usr_cmd_, 0, sizeof(UserCommand));
    }

    ~DdsCommandInterface() {
        Stop();
    }

    /**
     * @brief 设置 ROS 2 节点（需在 Start 前调用）
     * @param node  共享的 rclcpp 节点指针
     */
    void SetNode(rclcpp::Node::SharedPtr node) {
        node_ = node;
    }

    void Start() override {
        if (!node_) {
            std::cerr << "[DDS] Error: node not set! Call SetNode() first." << std::endl;
            return;
        }

        // 订阅 /STEER 话题（轴指令），参考 basic_server 发出端
        steer_sub_ = node_->create_subscription<drdds::msg::Steer>(
            "/STEER", 10,
            std::bind(&DdsCommandInterface::SteerCallback, this, std::placeholders::_1));

        // 订阅 /GAMEPAD_KEY 话题（按键），参考 basic_server 发出端
        key_sub_ = node_->create_subscription<std_msgs::msg::String>(
            "/GAMEPAD_KEY", 10,
            std::bind(&DdsCommandInterface::KeyCallback, this, std::placeholders::_1));

        std::cout << "\n╔════════════════════════════════════════════════╗\n"
                  << "║              DDS COMMAND INTERFACE              ║\n"
                  << "╚════════════════════════════════════════════════╝\n"
                  << "  Topics:\n"
                  << "    /STEER      → x=Forward, y=Side, yaw=Yaw\n"
                  << "    /GAMEPAD_KEY → key name string\n"
                  << "  Keys: L1=Stand  L2=RL Ctrl  R1=Lie Down  R2=Damping\n"
                  << "\n";
    }

    void Stop() override {
        if (steer_sub_) steer_sub_.reset();
        if (key_sub_) key_sub_.reset();
        usr_cmd_->forward_vel_scale = 0.0f;
        usr_cmd_->side_vel_scale = 0.0f;
        usr_cmd_->turnning_vel_scale = 0.0f;
    }

    UserCommand* GetUserCommand() override {
        return usr_cmd_;
    }
};

// 静态成员初始化（G20 + G12 兼容映射）
std::unordered_map<std::string, KeyCode> DdsCommandInterface::keyMap = {
    {"G20_KEY_L1", KeyCode::L1},
    {"G20_KEY_L2", KeyCode::L2},
    {"G20_KEY_R1", KeyCode::R1},
    {"G20_KEY_R2", KeyCode::R2},
    {"G12_KEY_C", KeyCode::L1},
    {"G12_KEY_A", KeyCode::L2},
    {"G12_KEY_B", KeyCode::R1},
    {"G12_KEY_D", KeyCode::R2}};
