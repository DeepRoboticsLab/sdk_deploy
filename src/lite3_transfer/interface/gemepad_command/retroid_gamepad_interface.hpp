/**
 * @file retroid_gamepad_interface.hpp
 * @brief this file is used for retroid gamepad command udp interface
 * @author Haokai Dai
 * @version 1.0
 * @date 2025-12-30
 * 
 * @copyright Copyright (c) 2025  DeepRobotics
 * 
 */
#pragma once

#include "gamepad_interface.h"
#include "retroid_gamepad.h"
#include <chrono>
#include <iostream>

/**
 * @brief Retroid gamepad implementation
 * @details This class implements the GamepadInterface for Retroid gamepads
 *          It reads gamepad data via UDP and publishes it as ROS2 messages
 */
class RetroidGamepadPublisher : public GamepadInterface {
private:
    std::shared_ptr<RetroidGamepad> gamepad_ptr_;
    int port_;

public:
    /**
     * @brief Constructor
     * @param port UDP port to receive gamepad data (default: 12121)
     */
    RetroidGamepadPublisher(int port = 12121) 
        : GamepadInterface("retroid_gamepad"),
          port_(port) {
        RCLCPP_INFO(this->get_logger(), "Using Retroid Gamepad Publisher on port %d", port);
        gamepad_ptr_ = std::make_shared<RetroidGamepad>(port);
    }

    virtual ~RetroidGamepadPublisher() {
        Stop();
    }

    /**
     * @brief Start the gamepad interface
     * @details Starts the gamepad data thread and publishing thread
     */
    void Start() override {
        if (running_) return;
        running_ = true;
        gamepad_ptr_->StartDataThread();
        publish_thread_ = std::thread(&RetroidGamepadPublisher::PublishGamepadData, this);
        RCLCPP_INFO(this->get_logger(), "Retroid gamepad publisher started");
    }

    /**
     * @brief Stop the gamepad interface
     * @details Stops all threads and cleans up resources
     */
    void Stop() override {
        if (!running_) return;
        running_ = false;
        gamepad_ptr_->StopDataThread();
        if (publish_thread_.joinable()) {
            publish_thread_.join();
        }
        RCLCPP_INFO(this->get_logger(), "Retroid gamepad publisher stopped");
    }

    /**
     * @brief Get the current gamepad data
     * @return GamepadData message containing current gamepad state
     * @details Reads raw gamepad data from RetroidGamepad and converts it to GamepadData format
     */
    drdds::msg::GamepadData GetGamepadData() override {
        RetroidKeys rt_keys_ = gamepad_ptr_->GetKeys();
        
        drdds::msg::GamepadData msg;
        
        // 设置摇杆值
        msg.left_axis_x = rt_keys_.left_axis_x;
        msg.left_axis_y = rt_keys_.left_axis_y;
        msg.right_axis_x = rt_keys_.right_axis_x;
        msg.right_axis_y = rt_keys_.right_axis_y;
        
        // 设置按钮位掩码
        msg.buttons = rt_keys_.value;
        
        return msg;
    }
};


