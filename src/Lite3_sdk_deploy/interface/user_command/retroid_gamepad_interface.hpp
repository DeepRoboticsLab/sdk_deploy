/**
 * @file retroid_gamepad_interface.hpp
 * @brief this file is used for retroid gamepad user command input
 * @author Haokai Dai
 * @version 1.0
 * @date 2025-12-30
 * 
 * @copyright Copyright (c) 2025  DeepRobotics
 * 
 */
#pragma once

#include "user_command_interface.h"
#include "custom_types.h"
#include "drdds/msg/gamepad_data.hpp"
#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <atomic>
#include <stdexcept>
#include <cstring>

using namespace interface;
using namespace types;

// RetroidKeys 位掩码定义（从 gamepad_keys.h 提取）
// 位顺序：R1, L1, start, select, R2, L2, A, B, X, Y, left, right, up, down, left_axis_button, right_axis_button
namespace {
    constexpr uint16_t BIT_R1 = (1 << 0);
    constexpr uint16_t BIT_L1 = (1 << 1);
    constexpr uint16_t BIT_START = (1 << 2);
    constexpr uint16_t BIT_SELECT = (1 << 3);
    constexpr uint16_t BIT_R2 = (1 << 4);
    constexpr uint16_t BIT_L2 = (1 << 5);
    constexpr uint16_t BIT_A = (1 << 6);
    constexpr uint16_t BIT_B = (1 << 7);
    constexpr uint16_t BIT_X = (1 << 8);
    constexpr uint16_t BIT_Y = (1 << 9);
    constexpr uint16_t BIT_LEFT = (1 << 10);
    constexpr uint16_t BIT_RIGHT = (1 << 11);
    constexpr uint16_t BIT_UP = (1 << 12);
    constexpr uint16_t BIT_DOWN = (1 << 13);
    constexpr uint16_t BIT_LEFT_AXIS_BUTTON = (1 << 14);
    constexpr uint16_t BIT_RIGHT_AXIS_BUTTON = (1 << 15);
}

class RetroidGamepadInterface : public UserCommandInterface {
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<drdds::msg::GamepadData>::SharedPtr gamepad_sub_;
    std::mutex cmd_mutex_;
    std::atomic<bool> running_{false};
    
    uint16_t last_buttons_ = 0;
    bool first_message_ = true;

    bool IsKeysEqual(uint16_t a, uint16_t b) {
        return a == b;
    }

    void GamepadDataCallback(const drdds::msg::GamepadData::SharedPtr msg) {
        static bool first_callback = true;
        if (first_callback) {
            RCLCPP_INFO(node_->get_logger(), "First message received! Gamepad Subscription is working.");
            first_callback = false;
        }
        
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        
        // 更新速度指令（始终更新，不依赖按键变化）
        usr_cmd_->forward_vel_scale = msg->left_axis_y;
        usr_cmd_->side_vel_scale = -msg->left_axis_x;
        usr_cmd_->turnning_vel_scale = -msg->right_axis_x;
        
        // 处理按键变化（模式切换）
        if (first_message_) {
            last_buttons_ = msg->buttons;
            first_message_ = false;
            return;
        }

        if (!IsKeysEqual(msg->buttons, last_buttons_)) {
            // 提取按键状态（从位掩码中提取）
            bool Y_pressed = (msg->buttons & BIT_Y) != 0;
            bool Y_last = (last_buttons_ & BIT_Y) != 0;
            bool A_pressed = (msg->buttons & BIT_A) != 0;
            bool A_last = (last_buttons_ & BIT_A) != 0;
            bool left_axis_button = (msg->buttons & BIT_LEFT_AXIS_BUTTON) != 0;
            bool right_axis_button = (msg->buttons & BIT_RIGHT_AXIS_BUTTON) != 0;
            bool left_axis_button_last = (last_buttons_ & BIT_LEFT_AXIS_BUTTON) != 0;
            bool right_axis_button_last = (last_buttons_ & BIT_RIGHT_AXIS_BUTTON) != 0;

            // 状态机模式切换逻辑
            if (msfb_ != nullptr) {
                switch (msfb_->GetCurrentState()) {
                    case RobotMotionState::WaitingForStand:
                        if (Y_pressed && !Y_last) {
                            usr_cmd_->target_mode = uint8_t(RobotMotionState::StandingUp);
                            RCLCPP_INFO(node_->get_logger(), "Mode: Standing Up");
                        }
                        break;
                    case RobotMotionState::StandingUp:
                        if (A_pressed && !A_last) {
                            usr_cmd_->target_mode = uint8_t(RobotMotionState::RLControlMode);
                            RCLCPP_INFO(node_->get_logger(), "Mode: RL Control");
                        }
                        break;
                    default:
                        break;
                }
            }

            // 双摇杆按键 -> Joint Damping
            if (left_axis_button && right_axis_button) {
                usr_cmd_->target_mode = uint8_t(RobotMotionState::JointDamping);
                RCLCPP_INFO(node_->get_logger(), "Mode: Joint Damping");
            }

            last_buttons_ = msg->buttons;
        }

        // 更新时间戳
        usr_cmd_->time_stamp = rclcpp::Time(msg->header.stamp).seconds();
    }

public:
    RetroidGamepadInterface(RobotName robot_name, rclcpp::Node::SharedPtr node) 
        : UserCommandInterface(robot_name) {
        if (!node) {
            std::cerr << "[RetroidGamepadInterface] Error: node cannot be nullptr!" << std::endl;
            throw std::invalid_argument("RetroidGamepadInterface requires a valid ROS2 node");
        }
        
        node_ = node;
        RCLCPP_INFO(node_->get_logger(), "Initialized, using node: %s", node_->get_name());
        
        // 创建订阅者
        gamepad_sub_ = node_->create_subscription<drdds::msg::GamepadData>(
            "/GAMEPAD_DATA",
            10,
            std::bind(&RetroidGamepadInterface::GamepadDataCallback, this, std::placeholders::_1)
        );
        
        RCLCPP_INFO(node_->get_logger(), "Subscribed to /GAMEPAD_DATA on node: %s", node_->get_name());
        // RCLCPP_INFO(node_->get_logger(), "Subscription created successfully, waiting for messages...");
        
        std::memset(usr_cmd_, 0, sizeof(UserCommand));
        last_buttons_ = 0;
    }

    ~RetroidGamepadInterface() {
        Stop();
    }

    void Start() override {
        if (running_) return;
        running_ = true;
        RCLCPP_INFO(node_->get_logger(), "Started");
    }

    void Stop() override {
        if (!running_) return;
        running_ = false;
        
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        usr_cmd_->forward_vel_scale = 0.0f;
        usr_cmd_->side_vel_scale = 0.0f;
        usr_cmd_->turnning_vel_scale = 0.0f;
        
        RCLCPP_INFO(node_->get_logger(), "Stopped");
    }

    UserCommand* GetUserCommand() override {
        return usr_cmd_;
    }

    rclcpp::Node::SharedPtr get_node() {
        return node_;
    }
};

