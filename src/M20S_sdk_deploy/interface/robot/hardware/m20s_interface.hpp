#pragma once

#include "robot_interface.h"
#include "dds_interface.hpp"

class M20SInterface : public DdsInterface {
protected:
    void ResetPositionOffset() {
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
        VecXf current_joint_pos = this->GetJointPosition();

        for (int i = 1; i < dof_num_; i += 4) {
            if (current_joint_pos(i) < -140. / 180. * M_PI) {
                pos_offset_[i] = pos_offset_[i] + 360.;
                std::cout << "joint " << i << " offset is changed to " << pos_offset_[i] << "\n";
            } else if (current_joint_pos(i) > 140. / 180. * M_PI) {
                pos_offset_[i] = pos_offset_[i] - 360.;
                std::cout << "joint " << i << " offset is changed to " << pos_offset_[i] << "\n";
            }
        }
        for (int i = 2; i < dof_num_; i += 4) {
            if (current_joint_pos(i) < -164. / 180. * M_PI) {
                pos_offset_[i] = pos_offset_[i] + 360.;
                std::cout << "joint " << i << " offset is changed to " << pos_offset_[i] << "\n";
            } else if (current_joint_pos(i) > 164. / 180. * M_PI) {
                pos_offset_[i] = pos_offset_[i] - 360.;
                std::cout << "joint " << i << " offset is changed to " << pos_offset_[i] << "\n";
            }
        }
        for (int i = 0; i < dof_num_; ++i) {
            joint_config_[i].dir = joint_dir_[i];
            joint_config_[i].offset = Deg2Rad(pos_offset_[i]);
        }
    }

public:
    M20SInterface(const std::string &robot_name) : DdsInterface(robot_name, 16) {
        battery_data_.resize(2 * BATTERY_DATA_SIZE);

        float init_pos_offset[16] = {-25, -131, 160, 0.,
                                     25, -131, 160, 0,
                                     -25, 131, -160, 0,
                                     25, 131, -160, 0};
        float joint_dir[16] = {1, 1, -1, 1,
                               1, -1, 1, -1,
                               -1, 1, -1, 1.,
                               -1, -1, 1, -1};
        for (int i = 0; i < dof_num_; ++i) {
            pos_offset_[i] = init_pos_offset[i];
            joint_dir_[i] = joint_dir[i];
            data_updated_[i] = false;
            joint_config_[i].dir = joint_dir_[i];
            joint_config_[i].offset = Deg2Rad(pos_offset_[i]);
        }
    }

    ~M20SInterface() {
    }

    virtual void Start() {
        time_stamp_ = GetTimestampMs();
        ResetPositionOffset();
    }

    virtual void Stop() {
    }

    /**
     * Sends joint commands to the robot.
     * Input matrix layout: [dof × 5] → [kp, pos, kd, vel, torque_ff]
     * 
     * ⚠ Torque values (column 4) are protected by low-level hardware torque limits
     *    → commands exceeding limits are saturated at the firmware/hardware level
     * 
     * Joints with index % 4 == 3 have kp forced to 0 (velocity/torque mode)
     */
    virtual void SetJointCommand(Eigen::Matrix<float, Eigen::Dynamic, 5> input) {
        auto msg = drdds::msg::JointsDataCmd();
        for (int i = 0; i < dof_num_; ++i) {
            msg.data.joints_data[i].position =
                    (input(i, 1) - joint_config_[i].offset) * joint_config_[i].dir;
            msg.data.joints_data[i].velocity = input(i, 3) * joint_dir_[i];
            msg.data.joints_data[i].torque = input(i, 4) * joint_dir_[i];
            msg.data.joints_data[i].kp = input(i, 0);
            if (i % 4 == 3) {
                msg.data.joints_data[i].kp = 0;
            }
            msg.data.joints_data[i].kd = input(i, 2);
            msg.data.joints_data[i].control_word = kIndexMotorControl;
        }
        joint_cmd_pub_->publish(msg);

    }

};
