#pragma once

#include "robot_interface.h"
#include "dds_interface.hpp"

class Lite3Interface : public DdsInterface {
protected:
    void ResetPositionOffset() override {
        memset(data_updated_, 0, dof_num_ * sizeof(bool));
        this->SetJointCommand(MatXf::Zero(dof_num_, 5));

        VecXf last_joint_pos = this->GetJointPosition();
        VecXf current_joint_pos = this->GetJointPosition();
        int cnt = 0;
        while (!IsDataUpdatedFinished()) {
            ++cnt;
            usleep(1000);

            rclcpp::spin_some(this->get_node());
            current_joint_pos = this->GetJointPosition();
            for (int i = 0; i < dof_num_; ++i) {
                if (!data_updated_[i] && current_joint_pos(i) != last_joint_pos(i) &&
                    !std::isnan(current_joint_pos(i))) {
                    data_updated_[i] = true;
                    std::cout << "joint " << i << " data updated at " << cnt << " cnt!" << std::endl;
                }
            }
            last_joint_pos = current_joint_pos;

            if (cnt > 10000) {
                for (int i = 0; i < dof_num_; ++i) {
                    std::cout << i << " :" << data_updated_[i] << std::endl;
                }
                std::cout << "joint data update is not finished\n";
            }
        }
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
    }

    virtual void SetJointCommand(Eigen::Matrix<float, Eigen::Dynamic, 5> input) {
        auto msg = drdds::msg::JointsDataCmd();
        msg.header.stamp = node_->now();  // 设置时间戳，避免延迟计算异常
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

