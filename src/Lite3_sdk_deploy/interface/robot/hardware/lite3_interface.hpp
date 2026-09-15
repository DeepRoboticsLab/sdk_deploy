#pragma once

#include "robot_interface.h"
#include "dds_interface.hpp"

class Lite3Interface : public DdsInterface {
protected:
    void ResetPositionOffset() override {
        memset(data_updated_, 0, dof_num_ * sizeof(bool));
        this->SetJointCommand(MatXf::Zero(dof_num_, 5));

        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
        while (rclcpp::ok() && (!IsDataUpdatedFinished() || !IsImuDataReady())) {
            rclcpp::spin_some(this->get_node());
            if (std::chrono::steady_clock::now() >= deadline)
                throw std::runtime_error("No valid joint/IMU data received within 10 seconds");
            usleep(1000);
        }
        if (!rclcpp::ok()) throw std::runtime_error("Robot startup interrupted");
    }

public:
    Lite3Interface(const std::string &robot_name) : DdsInterface(robot_name, 12) {
        battery_data_.resize(2 * BATTERY_DATA_SIZE);
        for (int i = 0; i < dof_num_; ++i) {
            data_updated_[i] = false;
        }
    }

    ~Lite3Interface() {
    }

    virtual void Start() {
        time_stamp_ = GetTimestampMs();
        ResetPositionOffset();
    }

    virtual void Stop() {
    }

// 在 Lite3Interface 类中添加
    virtual void Handler(const drdds::msg::JointsData::SharedPtr msg) override {
        std::lock_guard<std::mutex> lock(data_mutex_);
        topic_trace::LogEvent(
            node_->get_logger(),
            joints_data_sub_trace_,
            "SUB /JOINTS_DATA",
            rclcpp::Time(msg->header.stamp).seconds(),
            msg->header.frame_id);
        ++run_cnt_;
        for (int i = 0; i < dof_num_; ++i) {
            joint_pos_(i) = msg->data.joints_data[i].position;
            joint_vel_(i) = msg->data.joints_data[i].velocity;
            joint_tau_(i) = msg->data.joints_data[i].torque;
            motor_temperture_(i) = float(msg->data.joints_data[i].motion_temp);
            driver_temperture_(i) = float(msg->data.joints_data[i].driver_temp);
            driver_status_[i] = msg->data.joints_data[i].status_word;
            joint_data_id_[i] = uint16_t(run_cnt_);
        }
        ri_ts_ = rclcpp::Time(msg->header.stamp).seconds();
        for (int i = 0; i < dof_num_; ++i)
            data_updated_[i] = std::isfinite(joint_pos_(i)) && std::isfinite(joint_vel_(i));
    }

    virtual void SetJointCommand(Eigen::Matrix<float, Eigen::Dynamic, 5> input) {
        joint_cmd_ = input;
        auto msg = drdds::msg::JointsDataCmd();
        msg.header.frame_id = ++joints_cmd_pub_seq_;
        msg.header.stamp = node_->now();  // 设置时间戳，避免延迟计算异常
        topic_trace::LogEvent(
            node_->get_logger(),
            joints_cmd_pub_trace_,
            "PUB /JOINTS_CMD",
            rclcpp::Time(msg.header.stamp).seconds(),
            msg.header.frame_id);
        for (int i = 0; i < dof_num_; ++i) {
            msg.data.joints_data[i].position = input(i, 1);
            msg.data.joints_data[i].velocity = input(i, 3);
            msg.data.joints_data[i].torque = input(i, 4);
            msg.data.joints_data[i].kp = input(i, 0);
            msg.data.joints_data[i].kd = input(i, 2);
            msg.data.joints_data[i].control_word = kIndexMotorControl;
        }
        joint_cmd_pub_->publish(msg);

    }

};
