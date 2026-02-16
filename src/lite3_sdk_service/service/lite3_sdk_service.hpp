/**
 * @file lite3_sdk_service.hpp
 * @brief Lite3 SDK control service implementation
 * @author Haokai Dai
 * @version 1.0
 * @date 2026-01-23
 * 
 * @copyright Copyright (c) 2025  DeepRobotics
 * 
 */
#pragma once

#include "sdk_service.hpp"
#include "drdds/srv/std_srv_int32.hpp"

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <signal.h>
#include <sys/wait.h>
#include <errno.h>
#include <cstdlib>
#include <cstring>
#include <string>
#include <algorithm>
#include <chrono>
#include <thread>
#include <future>

/**
 * @brief Lite3 SDK control service implementation
 * @details This class implements the SdkService interface for controlling
 *          the lite3_transfer ROS2 node via UDP commands.
 */
class Lite3SdkService : public SdkService {
private:
    static constexpr size_t MAX_CMD_LENGTH = 256;  // Maximum command length
    static constexpr int32_t MIN_RATE = 1;
    static constexpr int32_t MAX_RATE = 200;
    static constexpr int32_t BASE_RATE = 200;
    static constexpr int STOP_TIMEOUT_MS = 2000;   // 2 seconds timeout for stop
    static constexpr int SERVICE_TIMEOUT_MS = 2000; // 2 seconds timeout for service call
    static constexpr size_t MAX_BUFFER_SIZE = 1024;  // Maximum UDP buffer size
    static constexpr int DEFAULT_UDP_PORT = 12122;   // Default UDP port

    pid_t lite3_pid_;
    rclcpp::Client<drdds::srv::StdSrvInt32>::SharedPtr client_;
    int udp_sockfd_;
    int udp_port_;

    /**
     * @brief Trim whitespace from string
     */
    static std::string trim(const std::string& input) {
        auto s = input;
        s.erase(s.begin(), std::find_if(s.begin(), s.end(),
            [](unsigned char ch) { return !std::isspace(ch); }));
        s.erase(std::find_if(s.rbegin(), s.rend(),
            [](unsigned char ch) { return !std::isspace(ch); }).base(), s.end());
        return s;
    }

    /**
     * @brief Check if string is a pure number
     */
    static bool is_number(const std::string& s) {
        if (s.empty()) return false;
        return std::all_of(s.begin(), s.end(), [](unsigned char c) {
            return std::isdigit(c);
        });
    }

    /**
     * @brief Check if process exists
     */
    static bool process_exists(pid_t pid) {
        if (pid <= 0) return false;
        return (kill(pid, 0) == 0);
    }

    /**
     * @brief Kill lite3_transfer process by name
     */
    static void pkill_lite3_transfer(int sig) {
        const char* cmd_int = "pkill -x -INT lite3_transfer >/dev/null 2>&1";
        const char* cmd_kill = "pkill -x -KILL lite3_transfer >/dev/null 2>&1";
        if (sig == SIGKILL) {
            (void)std::system(cmd_kill);
        } else {
            (void)std::system(cmd_int);
        }
    }

    /**
     * @brief Get client address as string for logging
     */
    static std::string get_client_address(const sockaddr_in& addr) {
        char ip_str[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &addr.sin_addr, ip_str, INET_ADDRSTRLEN);
        return std::string(ip_str) + ":" + std::to_string(ntohs(addr.sin_port));
    }

public:
    Lite3SdkService() : lite3_pid_(-1), udp_sockfd_(-1), udp_port_(DEFAULT_UDP_PORT) {}

    virtual ~Lite3SdkService() {
        Cleanup();
    }

    void Initialize(rclcpp::Node::SharedPtr node) override {
        node_ = node;
        client_ = node_->create_client<drdds::srv::StdSrvInt32>("/SDK_MODE");
        RCLCPP_INFO(get_logger(), "Lite3SdkService initialized");
    }

    bool StartNode() override {
        if (lite3_pid_ > 0 && process_exists(lite3_pid_)) {
            RCLCPP_WARN(get_logger(), "Node already running (PID: %d)", lite3_pid_);
            return true;
        }

        RCLCPP_INFO(get_logger(), "Starting lite3_transfer node...");
        lite3_pid_ = fork();
        
        if (lite3_pid_ < 0) {
            RCLCPP_ERROR(get_logger(), "fork() failed: %s", strerror(errno));
            lite3_pid_ = -1;
            return false;
        }

        if (lite3_pid_ == 0) {
            // Child process: set process group and exec ros2 command
            setpgid(0, 0);
            execlp("ros2", "ros2", "run", "lite3_transfer", "lite3_transfer", (char*)nullptr);
            RCLCPP_ERROR(get_logger(), "execlp() failed: %s", strerror(errno));
            _exit(1);
        }

        // Parent process: wait a bit to check if child started successfully
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (process_exists(lite3_pid_)) {
            RCLCPP_INFO(get_logger(), "lite3_transfer node started successfully (PID: %d)", lite3_pid_);
            return true;
        } else {
            RCLCPP_ERROR(get_logger(), "Child process exited immediately after fork");
            lite3_pid_ = -1;
            return false;
        }
    }

    bool StopNode() override {
        if (lite3_pid_ <= 0 && !process_exists(lite3_pid_)) {
            RCLCPP_DEBUG(get_logger(), "No node to stop (PID: %d)", lite3_pid_);
            // Try to kill by name anyway
            pkill_lite3_transfer(SIGINT);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            pkill_lite3_transfer(SIGKILL);
            return true;
        }

        RCLCPP_INFO(get_logger(), "Stopping lite3_transfer node (PID: %d)...", lite3_pid_);

        // First, try to gracefully exit by sending command 0 to /SDK_MODE service
        // This allows the node to properly switch to Robot mode before shutdown
        if (client_ && node_) {
            using namespace std::chrono_literals;
            if (client_->wait_for_service(1s)) {
                RCLCPP_INFO(get_logger(), "Sending command 0 to /SDK_MODE to exit SDK mode gracefully...");
                auto request = std::make_shared<drdds::srv::StdSrvInt32::Request>();
                request->command = 0;  // Command 0 means exit SDK mode
                
                auto future = client_->async_send_request(request);
                
                // Wait for response with timeout
                auto timeout = std::chrono::milliseconds(2000);
                auto start_time = std::chrono::steady_clock::now();
                auto check_interval = std::chrono::milliseconds(50);
                
                std::future_status status;
                bool service_success = false;
                while (true) {
                    if (node_) {
                        rclcpp::spin_some(node_);
                    }
                    
                    status = future.wait_for(check_interval);
                    if (status == std::future_status::ready) {
                        try {
                            auto response = future.get();
                            if (response && response->result == 0) {
                                service_success = true;
                                RCLCPP_INFO(get_logger(), "Successfully exited SDK mode via service");
                            }
                        } catch (const std::exception& e) {
                            RCLCPP_WARN(get_logger(), "Exception getting service response: %s", e.what());
                        }
                        break;
                    }
                    
                    auto elapsed = std::chrono::steady_clock::now() - start_time;
                    if (elapsed >= timeout) {
                        RCLCPP_WARN(get_logger(), "Service call timed out, proceeding with kill");
                        break;
                    }
                }
                
                // Give the node a moment to process the command and switch modes
                if (service_success) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                }
            } else {
                RCLCPP_WARN(get_logger(), "Service /SDK_MODE not available, proceeding with kill");
            }
        }

        // Now send SIGINT to allow graceful shutdown
        if (kill(-lite3_pid_, SIGINT) != 0) {
            RCLCPP_WARN(get_logger(), "kill(-%d, SIGINT) failed: %s", lite3_pid_, strerror(errno));
        }

        // Also try to kill by name
        pkill_lite3_transfer(SIGINT);

        // Non-blocking wait with timeout
        int status = 0;
        bool exited = false;
        auto start_time = std::chrono::steady_clock::now();
        
        while (std::chrono::steady_clock::now() - start_time < 
               std::chrono::milliseconds(STOP_TIMEOUT_MS)) {
            pid_t r = waitpid(lite3_pid_, &status, WNOHANG);
            if (r == lite3_pid_) {
                RCLCPP_INFO(get_logger(), "Node stopped successfully (exit status: %d)", status);
                exited = true;
                break;
            }
            if (r < 0) {
                if (errno == ECHILD) {
                    RCLCPP_INFO(get_logger(), "Process already exited");
                    exited = true;
                    break;
                }
                RCLCPP_WARN(get_logger(), "waitpid() failed: %s", strerror(errno));
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        if (!exited) {
            RCLCPP_WARN(get_logger(), "Node did not exit within timeout, forcing kill...");
            if (kill(-lite3_pid_, SIGKILL) != 0) {
                RCLCPP_WARN(get_logger(), "kill(-%d, SIGKILL) failed: %s", lite3_pid_, strerror(errno));
            }
            pkill_lite3_transfer(SIGKILL);
            
            // Final blocking wait
            if (waitpid(lite3_pid_, &status, 0) < 0) {
                RCLCPP_WARN(get_logger(), "Final waitpid() failed: %s", strerror(errno));
            } else {
                RCLCPP_INFO(get_logger(), "Node force-killed successfully");
            }
        }

        lite3_pid_ = -1;
        return true;
    }

    bool SetPublishRate(int32_t rate) override {
        // Validate rate
        if (rate <= 0 || rate > MAX_RATE || (BASE_RATE % rate != 0)) {
            RCLCPP_WARN(get_logger(), "Invalid publish rate: %d (must be > 0, <= %d, and divisor of %d)",
                       rate, MAX_RATE, BASE_RATE);
            return false;
        }

        if (!client_) {
            RCLCPP_ERROR(get_logger(), "Service client not initialized");
            return false;
        }

        RCLCPP_INFO(get_logger(), "Setting publish rate to %d Hz...", rate);

        // Wait for service with timeout (non-blocking check)
        using namespace std::chrono_literals;
        if (!client_->wait_for_service(1s)) {
            RCLCPP_ERROR(get_logger(), "Service /SDK_MODE not available");
            return false;
        }

        // Create request
        auto request = std::make_shared<drdds::srv::StdSrvInt32::Request>();
        request->command = rate;

        // Send request asynchronously
        auto future = client_->async_send_request(request);

        // Wait for response with timeout, periodically spinning to allow ROS2 to process callbacks
        auto timeout = std::chrono::milliseconds(SERVICE_TIMEOUT_MS);
        auto start_time = std::chrono::steady_clock::now();
        auto check_interval = std::chrono::milliseconds(50);  // Check every 50ms
        
        std::future_status status;
        while (true) {
            // Spin once to allow ROS2 to process service response
            if (node_) {
                rclcpp::spin_some(node_);
            }
            
            // Check if future is ready
            status = future.wait_for(check_interval);
            if (status == std::future_status::ready) {
                break;
            }
            
            // Check timeout
            auto elapsed = std::chrono::steady_clock::now() - start_time;
            if (elapsed >= timeout) {
                RCLCPP_ERROR(get_logger(), "Service call timed out after %d ms", SERVICE_TIMEOUT_MS);
                return false;
            }
        }

        // Get response
        try {
            auto response = future.get();
            if (!response) {
                RCLCPP_ERROR(get_logger(), "Empty response from /SDK_MODE");
                return false;
            }

            if (response->result != 0) {
                RCLCPP_WARN(get_logger(), "Set publish rate failed, response->result = %d", response->result);
                return false;
            }

            RCLCPP_INFO(get_logger(), "Publish rate set to %d Hz successfully", rate);
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Exception while getting service response: %s", e.what());
            return false;
        }
    }

    std::string ProcessCommand(const std::string& cmd) override {
        // Validate command length
        if (cmd.length() > MAX_CMD_LENGTH) {
            RCLCPP_WARN(get_logger(), "Command too long: %zu bytes (max: %zu)", cmd.length(), MAX_CMD_LENGTH);
            return "invalid";
        }

        // Trim and normalize command
        std::string trimmed = trim(cmd);
        if (trimmed.empty()) {
            RCLCPP_DEBUG(get_logger(), "Received empty command");
            return "invalid";
        }

        // Convert to lowercase
        std::string lower_cmd = trimmed;
        std::transform(lower_cmd.begin(), lower_cmd.end(), lower_cmd.begin(),
                      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

        RCLCPP_DEBUG(get_logger(), "Processing command: '%s'", lower_cmd.c_str());

        // Handle "on" command
        if (lower_cmd == "on") {
            bool ok = StartNode();
            RCLCPP_INFO(get_logger(), "Command 'on' result: %s", ok ? "success" : "failure");
            return ok ? "success" : "failure";
        }

        // Handle "off" command
        if (lower_cmd == "off") {
            bool ok = StopNode();
            RCLCPP_INFO(get_logger(), "Command 'off' result: %s", ok ? "success" : "failure");
            return ok ? "success" : "failure";
        }

        // Handle numeric command (rate)
        if (is_number(lower_cmd)) {
            try {
                int32_t rate = std::stoi(lower_cmd);
                
                // Validate rate range and divisor
                if (rate <= 0 || rate > MAX_RATE || (BASE_RATE % rate != 0)) {
                    RCLCPP_WARN(get_logger(), "Invalid rate value: %d", rate);
                    return "invalid";
                }

                bool ok = SetPublishRate(rate);
                RCLCPP_INFO(get_logger(), "Command rate=%d result: %s", rate, ok ? "success" : "failure");
                return ok ? "success" : "failure";
            } catch (const std::exception& e) {
                RCLCPP_ERROR(get_logger(), "Exception parsing rate command '%s': %s", lower_cmd.c_str(), e.what());
                return "invalid";
            }
        }

        // Unknown command
        RCLCPP_WARN(get_logger(), "Unknown command: '%s'", lower_cmd.c_str());
        return "invalid";
    }

    bool InitializeUdp(int port) override {
        udp_port_ = port;

        // Create UDP socket
        udp_sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (udp_sockfd_ < 0) {
            RCLCPP_ERROR(get_logger(), "Failed to create socket: %s", strerror(errno));
            return false;
        }

        // Set socket options for better error handling
        int reuse = 1;
        if (setsockopt(udp_sockfd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
            RCLCPP_WARN(get_logger(), "Failed to set SO_REUSEADDR: %s", strerror(errno));
        }

        // Bind to UDP port
        sockaddr_in addr;
        std::memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_port = htons(udp_port_);

        if (bind(udp_sockfd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
            RCLCPP_ERROR(get_logger(), "Failed to bind to port %d: %s", udp_port_, strerror(errno));
            close(udp_sockfd_);
            udp_sockfd_ = -1;
            return false;
        }

        RCLCPP_INFO(get_logger(), "UDP socket initialized, listening on port %d", udp_port_);
        return true;
    }

    void RunUdpLoop() override {
        if (udp_sockfd_ < 0) {
            RCLCPP_ERROR(get_logger(), "UDP socket not initialized, call InitializeUdp() first");
            return;
        }

        RCLCPP_INFO(get_logger(), "Starting UDP receive loop on port %d", udp_port_);

        char buffer[MAX_BUFFER_SIZE];
        while (rclcpp::ok()) {
            sockaddr_in remote_addr;
            socklen_t remote_len = sizeof(remote_addr);

            // Receive UDP packet
            ssize_t len = recvfrom(udp_sockfd_, buffer, sizeof(buffer) - 1, 0,
                                  reinterpret_cast<sockaddr*>(&remote_addr), &remote_len);

            if (len < 0) {
                if (errno == EINTR) {
                    // Interrupted by signal, continue
                    continue;
                }
                RCLCPP_ERROR(get_logger(), "recvfrom() failed: %s", strerror(errno));
                continue;
            }

            // Null-terminate and validate length
            if (len >= static_cast<ssize_t>(sizeof(buffer))) {
                RCLCPP_WARN(get_logger(), "Received packet too large: %zd bytes (max: %zu)",
                           len, sizeof(buffer) - 1);
                // Still process it, but truncate
                len = sizeof(buffer) - 1;
            }
            buffer[len] = '\0';

            // Validate command length
            if (len > static_cast<ssize_t>(MAX_CMD_LENGTH)) {
                RCLCPP_WARN(get_logger(), "Command too long: %zd bytes (max: %zu) from %s",
                           len, MAX_CMD_LENGTH, get_client_address(remote_addr).c_str());
                std::string reply = "invalid";
                sendto(udp_sockfd_, reply.c_str(), reply.size(), 0,
                       reinterpret_cast<sockaddr*>(&remote_addr), remote_len);
                continue;
            }

            std::string cmd(buffer, len);
            std::string client_addr = get_client_address(remote_addr);
            RCLCPP_DEBUG(get_logger(), "Received command from %s: '%s'", client_addr.c_str(), cmd.c_str());

            // Process command through service
            std::string reply;
            try {
                reply = ProcessCommand(cmd);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(get_logger(), "Exception processing command '%s': %s", cmd.c_str(), e.what());
                reply = "failure";
            }

            // Send response
            ssize_t sent = sendto(udp_sockfd_, reply.c_str(), reply.size(), 0,
                                 reinterpret_cast<sockaddr*>(&remote_addr), remote_len);
            if (sent < 0) {
                RCLCPP_ERROR(get_logger(), "sendto() failed: %s", strerror(errno));
            } else {
                RCLCPP_DEBUG(get_logger(), "Sent reply to %s: '%s'", client_addr.c_str(), reply.c_str());
            }

            // Spin ROS2 once to handle callbacks (non-blocking)
            if (node_) {
                rclcpp::spin_some(node_);
            }
        }

        RCLCPP_INFO(get_logger(), "UDP receive loop ended");
    }

    void Cleanup() override {
        RCLCPP_INFO(get_logger(), "Cleaning up Lite3SdkService...");
        StopNode();
        
        // Close UDP socket
        if (udp_sockfd_ >= 0) {
            close(udp_sockfd_);
            udp_sockfd_ = -1;
        }
        
        client_.reset();
        node_.reset();
    }
};
