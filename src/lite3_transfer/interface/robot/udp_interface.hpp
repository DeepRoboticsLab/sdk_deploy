/**
 * @file udp_interface.h
 * @brief this file is for applying robot's ros2-udp interface
 * @author Haokai Dai
 * @version 1.0
 * @date 2025-12-25
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "drdds/msg/joints_data.hpp"
#include "drdds/msg/joints_data_cmd.hpp"
#include "drdds/msg/imu_data.hpp"
#include "drdds/srv/std_srv_int32.hpp"
#include "robot_interface.h"
#include "robot_types.h"
#include "receiver.h"
#include "sender.h"

#include <thread>
#include <atomic>
#include <mutex>
#include <array>
#include <chrono>

// using namespace lite3;

class HardwareInterface : public interface::RobotInterface,
                          public rclcpp::Node
{
private:
    RobotData* robot_data_=nullptr;
    RobotCmd robot_joint_cmd_{};
    Receiver* receiver_ = nullptr;
    Sender* sender_ = nullptr;

    Vec3f omega_body_, rpy_, acc_;
    VecXf joint_pos_, joint_vel_, joint_tau_;
    std::thread hw_thread_;
    std::thread ros2_thread_;

    static constexpr std::array<const char*, 12> JOINT_NAMES = {
        "FL_HipX", "FL_HipY", "FL_Knee",
        "FR_HipX", "FR_HipY", "FR_Knee",
        "HL_HipX", "HL_HipY", "HL_Knee",
        "HR_HipX", "HR_HipY", "HR_Knee"
    };
    
    rclcpp::Subscription<drdds::msg::JointsDataCmd>::SharedPtr joint_cmd_sub_;
    rclcpp::Publisher<drdds::msg::JointsData>::SharedPtr joint_data_pub_;
    rclcpp::Publisher<drdds::msg::ImuData>::SharedPtr imu_data_pub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp::TimerBase::SharedPtr cmd_timer_;  // 1000Hz 定时器用于发送 UDP 命令
    rclcpp::Service<drdds::srv::StdSrvInt32>::SharedPtr set_rate_service_;
    std::mutex rate_mutex_;  // 保护 publish_rate_ 和 timer 的线程安全
    std::mutex cmd_mutex_;   // 保护 robot_joint_cmd_ 的线程安全

    // Service callback 函数
    void SetPublishRateCallback(
        const std::shared_ptr<drdds::srv::StdSrvInt32::Request> request,
        std::shared_ptr<drdds::srv::StdSrvInt32::Response> response)
    {
        int32_t command = request->command;
        
        // 如果输入频率为0，则调用Stop退出SDK模式切换至robot模式
        if (command == 0) {
            RCLCPP_INFO(get_logger(), "Command 0 received: Exiting SDK mode, switching to Robot mode");
            Stop();
            response->result = 0;  // 成功
            return;
        }
        
        // 判断输入数值：必须小于等于200且为1000的因数
        if (command < 0 || command > 200 || 1000 % command != 0) {
            RCLCPP_WARN(get_logger(), 
                "Invalid publish rate: %d Hz. The value must be less than or equal to 200 and a divisor of 1000. "
                "Valid values: 1, 2, 4, 5, 8, 10, 20, 25, 40, 50, 100, 125, 200", 
                command);
            response->result = -1;  // 失败
            return;
        }
        
        {
            std::lock_guard<std::mutex> lock(rate_mutex_);
            publish_rate_ = static_cast<double>(command);
            
            // 取消旧的 timer
            if (publish_timer_) {
                publish_timer_->cancel();
            }
            
            // 创建新的 timer 使用新的频率
            using namespace std::chrono_literals;
            auto period = std::chrono::duration<double>(1.0 / publish_rate_);
            publish_timer_ = this->create_wall_timer(
                std::chrono::duration_cast<std::chrono::milliseconds>(period),
                [this]() {
                    this->PublishJointData();
                    this->PublishImuData();
                }
            );
        }
        
        RCLCPP_INFO(get_logger(), "Publish rate set to %d Hz", command);
        response->result = 0;  // 成功
    }

public:
    // 将频率设置放到public中，可供外部查看频率
    double publish_rate_ = 200;
    HardwareInterface(const std::string& robot_name, 
                        int local_port=43897, 
                        std::string robot_ip="192.168.2.1",
                        int robot_port=43893):RobotInterface(robot_name, 12)
                        ,rclcpp::Node(robot_name){
        std::cout << robot_name << " is using Lite3 ROS2 Interface" << std::endl;
        // receiver_ = new Receiver(local_port);
        receiver_ = new Receiver();
        sender_ = new Sender(robot_ip, robot_port);
        sender_->RobotStateInit();

        joint_data_pub_ = create_publisher<drdds::msg::JointsData>("/JOINTS_DATA", 10);
        imu_data_pub_ = create_publisher<drdds::msg::ImuData>("/IMU_DATA", 10);
        joint_cmd_sub_ = create_subscription<drdds::msg::JointsDataCmd>("/JOINTS_CMD", 10,
                        std::bind(&HardwareInterface::SetJointCommand, this, std::placeholders::_1));
        
        // 创建 service server
        set_rate_service_ = create_service<drdds::srv::StdSrvInt32>(
            "/SDK_MODE",
            std::bind(&HardwareInterface::SetPublishRateCallback, this,
                     std::placeholders::_1, std::placeholders::_2));
        
        RCLCPP_INFO(get_logger(), "ROS2 interfaces initialized");
        RCLCPP_INFO(get_logger(), "  Publishing: /JOINTS_DATA, /IMU_DATA");
        RCLCPP_INFO(get_logger(), "  Subscribing: /JOINTS_CMD");
        RCLCPP_INFO(get_logger(), "  Service: /SDK_MODE (set publish rate, 0 to exit SDK mode)");

        using namespace std::chrono_literals;
        auto period = std::chrono::duration<double>(1.0 / publish_rate_);  // 1/1000 s
        publish_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            [this]() {
                // 每个周期同时发布关节和 IMU
                this->PublishJointData();
                this->PublishImuData();
            }
        );
        
        // 创建 1000Hz 定时器用于发送 UDP 命令
        // 每次收到 ROS 消息（200Hz）时，会通过这个定时器发送 5 次 UDP 消息（1000Hz）
        constexpr double cmd_send_rate = 1000.0;  // 1000 Hz
        auto cmd_period = std::chrono::duration<double>(1.0 / cmd_send_rate);  // 1ms
        cmd_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::microseconds>(cmd_period),
            [this]() {
                this->SendCommandTimerCallback();
            }
        );
        RCLCPP_INFO(get_logger(), "UDP command send timer initialized at %.0f Hz", cmd_send_rate);

    }   
    ~HardwareInterface(){
        Stop();
    }

    virtual void Start(){
        // RCLCPP_INFO(get_logger(), "  Starting receiver");
        receiver_->StartWork();                        
        robot_data_ = &(receiver_->GetState());
        if (sender_ != nullptr)
            sender_->ControlGet(2); ///< SDK mode
    }

    virtual void Stop(){
        if(sender_ != nullptr){
            sender_->ControlGet(1); ///< Robot mode
        }
    }

    virtual double GetInterfaceTimeStamp(){
        return robot_data_->tick*0.001;
    }
    virtual VecXf GetJointPosition() {
        joint_pos_ = VecXf::Zero(dof_num_);
        for(int i=0;i<dof_num_;++i){
            joint_pos_(i) = robot_data_->joint_data.joint_data[i].position;
        }
        return joint_pos_;
    };
    virtual VecXf GetJointVelocity() {
        joint_vel_ = VecXf::Zero(dof_num_);
        for(int i=0;i<dof_num_;++i){
            joint_vel_(i) = robot_data_->joint_data.joint_data[i].velocity;
        }
        return joint_vel_;
    }
    virtual VecXf GetJointTorque() {
        joint_tau_ = VecXf::Zero(dof_num_);
        for(int i=0;i<dof_num_;++i){
            joint_tau_(i) = robot_data_->joint_data.joint_data[i].torque;
        }
        return joint_tau_;
    }

    virtual Vec3f GetImuRpy() {
        rpy_ << robot_data_->imu.angle_roll/180.*M_PI, robot_data_->imu.angle_pitch/180.*M_PI, robot_data_->imu.angle_yaw/180.*M_PI;
        return rpy_;
    }

    virtual Vec3f GetImuAcc() {
        acc_ << robot_data_->imu.acc_x, robot_data_->imu.acc_y, robot_data_->imu.acc_z;
        return acc_;
    }

    virtual Vec3f GetImuOmega() {
        omega_body_ << robot_data_->imu.angular_velocity_roll, robot_data_->imu.angular_velocity_pitch, robot_data_->imu.angular_velocity_yaw;
        return omega_body_;
    }
    virtual VecXf GetContactForce() {
        return VecXf::Zero(4);
    }
    
    // 定时器回调函数：以 1000Hz 频率发送 UDP 命令
    void SendCommandTimerCallback() {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        if (sender_ != nullptr) {
            sender_->SendCmd(robot_joint_cmd_);
        }
    }

    virtual void SetJointCommand(const drdds::msg::JointsDataCmd::SharedPtr msg){
        // 更新命令数据（加锁保护，避免与定时器回调冲突）
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        for(int i=0;i<dof_num_;++i){
            robot_joint_cmd_.joint_cmd[i].kp       = msg->data.joints_data[i].kp;
            robot_joint_cmd_.joint_cmd[i].position = msg->data.joints_data[i].position;
            robot_joint_cmd_.joint_cmd[i].kd       = msg->data.joints_data[i].kd;
            robot_joint_cmd_.joint_cmd[i].velocity = msg->data.joints_data[i].velocity;
            robot_joint_cmd_.joint_cmd[i].torque   = msg->data.joints_data[i].torque;
        }
        // 不再直接发送，由定时器以 1000Hz 频率发送
        // 这样每次收到 ROS 消息（200Hz）时，定时器会在 5ms 内发送 5 次 UDP 消息
    }
    virtual void PublishJointData(){
        auto pos = GetJointPosition();
        auto vel = GetJointVelocity();
        auto tau = GetJointTorque();

        drdds::msg::JointsData msg;
        msg.header.stamp = this->now();
        // msg.header.frame_id = 0;
        for(int i=0;i<dof_num_;++i){
            // msg.data.joints_data[i].name = JOINT_NAMES[i];
            msg.data.joints_data[i].status_word = 1;
            msg.data.joints_data[i].position = pos(i);
            msg.data.joints_data[i].velocity = vel(i);
            msg.data.joints_data[i].torque = tau(i);
        }
        joint_data_pub_->publish(msg);
    }
    virtual void PublishImuData(){
        auto rpy = GetImuRpy();
        auto omega = GetImuOmega();
        auto acc = GetImuAcc();

        drdds::msg::ImuData msg;
        msg.header.stamp = this->now();
        // msg.header.frame_id = 0;
        msg.data.roll = rpy(0);
        msg.data.pitch = rpy(1);
        msg.data.yaw = rpy(2);
        msg.data.omega_x = omega(0);
        msg.data.omega_y = omega(1);
        msg.data.omega_z = omega(2);
        msg.data.acc_x = acc(0);
        msg.data.acc_y = acc(1);
        msg.data.acc_z = acc(2);

        imu_data_pub_->publish(msg);
    }
};


