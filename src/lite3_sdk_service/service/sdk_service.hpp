/**
 * @file sdk_service.hpp
 * @brief Pure virtual base class for SDK control service
 * @author Haokai Dai
 * @version 1.0
 * @date 2026-01-23
 * 
 * @copyright Copyright (c) 2025  DeepRobotics
 * 
 */
#pragma once

#include "rclcpp/rclcpp.hpp"
#include <string>

/**
 * @brief Pure virtual base class for SDK control service
 * @details This class defines the interface for controlling SDK nodes,
 *          including starting/stopping nodes and setting publish rates.
 */
class SdkService {
public:
    virtual ~SdkService() = default;

protected:
    /**
     * @brief Protected default constructor
     * @details Allows derived classes to be constructed without initializing node_,
     *          which will be initialized in Initialize() method.
     */
    SdkService() = default;

    /**
     * @brief Initialize the service with a ROS2 node
     * @param node Shared pointer to ROS2 node
     */
    virtual void Initialize(rclcpp::Node::SharedPtr node) = 0;

    /**
     * @brief Start the SDK node
     * @return true if successful, false otherwise
     */
    virtual bool StartNode() = 0;

    /**
     * @brief Stop the SDK node
     * @return true if successful, false otherwise
     */
    virtual bool StopNode() = 0;

    /**
     * @brief Set the publish rate for the SDK node
     * @param rate Publish rate in Hz (must be > 0, <= 200, and a divisor of 1000)
     * @return true if successful, false otherwise
     */
    virtual bool SetPublishRate(int32_t rate) = 0;

    /**
     * @brief Process a UDP command and return response
     * @param cmd Command string (e.g., "on", "off", or a number)
     * @return Response string ("success", "failure", or "invalid")
     */
    virtual std::string ProcessCommand(const std::string& cmd) = 0;

    /**
     * @brief Initialize UDP socket and bind to port
     * @param port UDP port to listen on
     * @return true if successful, false otherwise
     */
    virtual bool InitializeUdp(int port) = 0;

    /**
     * @brief Run the UDP receive loop (blocking until rclcpp::ok() is false)
     */
    virtual void RunUdpLoop() = 0;

    /**
     * @brief Cleanup resources
     */
    virtual void Cleanup() = 0;

protected:
    rclcpp::Node::SharedPtr node_;

    /**
     * @brief Get logger from node
     * @return Logger instance
     * @details Returns logger from node_ if available, otherwise returns a default logger
     */
    rclcpp::Logger get_logger() const {
        if (node_) {
            return node_->get_logger();
        }
        // Return a default logger if node_ is not initialized
        static rclcpp::Logger default_logger = rclcpp::get_logger("sdk_service");
        return default_logger;
    }
};
