/**
 * @file gamepad_interface.h
 * @brief this file is used for gamepad command udp interface
 * @author Haokai Dai
 * @version 1.0
 * @date 2025-12-30
 * 
 * @copyright Copyright (c) 2025  DeepRobotics
 * 
 */
#pragma once

#include <rclcpp/rclcpp.hpp>
#include "drdds/msg/gamepad_data.hpp"
#include <thread>
#include <atomic>
#include <memory>
#include <chrono>

/**
 * @brief Base class for gamepad interfaces
 * @details This class provides a common interface for different gamepad implementations
 *          All gamepad interfaces should inherit from this class and implement the pure virtual functions
 */
class GamepadInterface : public rclcpp::Node {
protected:
    rclcpp::Publisher<drdds::msg::GamepadData>::SharedPtr gamepad_pub_;
    std::thread publish_thread_;
    std::atomic<bool> running_{false};

public:
    /**
     * @brief Constructor
     * @param node_name Name of the ROS2 node
     */
    GamepadInterface(const std::string& node_name) 
        : Node(node_name) {
        // 创建发布者
        gamepad_pub_ = this->create_publisher<drdds::msg::GamepadData>("/GAMEPAD_DATA", 10);
        RCLCPP_INFO(this->get_logger(), "GamepadInterface initialized, publishing to /GAMEPAD_DATA");
    }

    virtual ~GamepadInterface() {

    }

    /**
     * @brief Start the gamepad interface
     * @details This function should start the gamepad data thread and publishing thread
     */
    virtual void Start() = 0;

    /**
     * @brief Stop the gamepad interface
     * @details This function should stop all threads and clean up resources
     */
    virtual void Stop() = 0;

    /**
     * @brief Get the current gamepad data
     * @return GamepadData message containing current gamepad state
     * @details This function should be implemented to read raw gamepad data
     *          and convert it to GamepadData message format
     */
    virtual drdds::msg::GamepadData GetGamepadData() = 0;

    /**
     * @brief Publish gamepad data
     * @details This function publishes the gamepad data to the /GAMEPAD_DATA topic
     *          It can be overridden if custom publishing logic is needed
     */
    virtual void PublishGamepadData() {
        while (running_) {
            auto msg = GetGamepadData();
            msg.header.stamp = this->now();            
            gamepad_pub_->publish(msg);
            
            std::this_thread::sleep_for(std::chrono::milliseconds(5)); // 200Hz 发布频率
        }
    }
};
