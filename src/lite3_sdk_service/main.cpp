/**
 * @file main.cpp
 * @brief Main entry point for SDK control service
 * @author Haokai Dai
 * @version 1.0
 * @date 2026-01-23
 * 
 * @copyright Copyright (c) 2025  DeepRobotics
 * 
 */
#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "service/lite3_sdk_service.hpp"

int main(int argc, char** argv) {
    // Set default ROS_DOMAIN_ID to 0 if not set
    const char* domain_env = std::getenv("ROS_DOMAIN_ID");
    if (domain_env == nullptr) {
        setenv("ROS_DOMAIN_ID", "0", 1);
    }

    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Create ROS2 node
    auto node = rclcpp::Node::make_shared("lite3_sdk_service");
    RCLCPP_INFO(node->get_logger(), "Lite3 SDK service node starting...");

    // Create and initialize service
    Lite3SdkService service;
    try {
        service.Initialize(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(node->get_logger(), "Failed to initialize service: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    // Initialize UDP socket
    if (!service.InitializeUdp(12122)) {
        RCLCPP_FATAL(node->get_logger(), "Failed to initialize UDP socket");
        service.Cleanup();
        rclcpp::shutdown();
        return 1;
    }

    // Run UDP receive loop (blocking until rclcpp::ok() is false)
    service.RunUdpLoop();

    // Cleanup
    RCLCPP_INFO(node->get_logger(), "Shutting down...");
    service.Cleanup();
    rclcpp::shutdown();

    return 0;
}
