/**
 * @file dds_hardware_interface.hpp
 * @brief hardware interface for dds
 * @author DeepRobotics
 * @version 1.0
 * @date 2025-11-07
 * 
 * @copyright Copyright (c) 2025  DeepRobotics
 * 
 */

#pragma once


#include "common_types.h"
#include "robot_interface.h"
#include "dds_types.h"

#include "drdds/msg/joints_data.hpp"
#include "drdds/msg/joints_data_cmd.hpp"
#include "drdds/msg/imu_data.hpp"
#include "drdds/msg/battery_data.hpp"

using namespace dds;

struct JointConfig {
    float dir;
    float offset;
};


class DdsInterface : public RobotInterface{
protected:
    double ri_ts_, imu_ts_;
    Vec3f omega_body_, rpy_, acc_;
    VecXf joint_pos_, joint_vel_, joint_tau_;
    VecXf motor_temperture_, driver_temperture_;
    float *pos_offset_;
    float *joint_dir_;
    bool *data_updated_;
    uint16_t *control_word;

    JointConfig *joint_config_;
    std::vector<uint16_t> driver_status_;
    std::vector<uint16_t> joint_data_id_;
    BatteryInfo_dds battery_info_[2];
    std::vector<uint16_t> battery_data_;

    double time_stamp_;
    float remain_bat;

    rclcpp::Publisher<drdds::msg::JointsDataCmd>::SharedPtr joint_cmd_pub_;
    rclcpp::Subscription<drdds::msg::JointsData>::SharedPtr joint_data_sub_;
    rclcpp::Subscription<drdds::msg::ImuData>::SharedPtr imu_data_sub_;
    rclcpp::Subscription<drdds::msg::BatteryData>::SharedPtr health_data_sub_;


    bool IsDataUpdatedFinished() {
        bool res = data_updated_[0];
        for (int i = 0; i < dof_num_; ++i) {
            res = res & data_updated_[i];
        }
        return res;
    }

    void ResetPositionOffset() {
        memset(data_updated_, 0, dof_num_ * sizeof(bool));
        this->SetJointCommand(MatXf::Zero(dof_num_, 5));
        usleep(100 * 1000);
        VecXf last_joint_pos = this->GetJointPosition();
        VecXf current_joint_pos = this->GetJointPosition();
        int cnt = 0;
        while (!IsDataUpdatedFinished()) {
            ++cnt;
            this->RefreshRobotData();
            rclcpp::spin_some(this->get_node());
            current_joint_pos = this->GetJointPosition();

            for (int i = 0; i < dof_num_; ++i) {
                if (!data_updated_[i] && current_joint_pos(i) != last_joint_pos(i)) {
                    data_updated_[i] = true;
                }
            }
            last_joint_pos = current_joint_pos;
            usleep(1000);
            if (cnt == 10000) {
                std::cout << data_updated_ << std::endl;
                std::cout << "joint data update is not finished\n";
            }
        }
        for (int i = 1; i < dof_num_; i += 4) {
            if (current_joint_pos(i) < -210. / 180. * M_PI) {
                pos_offset_[i] = pos_offset_[i] + 360.;
                std::cout << "joint " << i << " offset is changed to " << pos_offset_[i] << "\n";
            } else if (current_joint_pos(i) > 90. / 180. * M_PI) {
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
    int run_cnt_ = 0;

    DdsInterface(const std::string &robot_name, const int &dof_num) : RobotInterface(robot_name, dof_num) {
        std::cout << robot_name << " is using Ecan Hardware Interface" << std::endl;
        joint_pos_ = VecXf::Zero(dof_num_);
        joint_vel_ = VecXf::Zero(dof_num_);
        joint_tau_ = VecXf::Zero(dof_num_);
        motor_temperture_ = VecXf::Zero(dof_num_);
        driver_temperture_ = VecXf::Zero(dof_num_);

        pos_offset_ = new float[dof_num_];
        joint_dir_ = new float[dof_num_];
        data_updated_ = new bool[dof_num_];
        joint_config_ = new JointConfig[dof_num_];
        control_word = new uint16_t[4];

        driver_status_.resize(dof_num_);
        joint_data_id_.resize(dof_num_);
        battery_data_.resize(BATTERY_DATA_SIZE);

        joint_cmd_pub_ = node_->create_publisher<drdds::msg::JointsDataCmd>("/JOINTS_CMD", 10);
        joint_data_sub_ = node_->create_subscription<drdds::msg::JointsData>("/JOINTS_DATA", 10,
                                    std::bind(&DdsInterface::Handler, this,std::placeholders::_1));
        imu_data_sub_ = node_->create_subscription<drdds::msg::ImuData>("/IMU_DATA", 10,
                                    std::bind(&DdsInterface::HandlerIMU, this, std::placeholders::_1));
        health_data_sub_ = node_->create_subscription<drdds::msg::BatteryData>("/BATTERY_DATA", 10,
                                    std::bind(&DdsInterface::HandlerHealth, this, std::placeholders::_1));
        
        sleep(1);

        control_word[0] = kIndexDisable;
        control_word[1] = kIndexErrorReset;
        control_word[2] = kIndexEnable;
        control_word[3] = kIndexGetStatusWord;
        ResetJointError();
    }

    virtual ~DdsInterface() {
        delete[] pos_offset_;
        delete[] joint_dir_;
        delete[] data_updated_;
        delete[] joint_config_;
    }

    virtual void Start() {
    }

    virtual void Stop() {
    }

    virtual double GetInterfaceTimeStamp() {
        return ri_ts_;
    }

    virtual VecXf GetJointPosition() {
        return joint_pos_;
    }

    virtual VecXf GetJointVelocity() {
        return joint_vel_;
    }

    virtual VecXf GetJointTorque() {
        return joint_tau_;
    }

    virtual Vec3f GetImuRpy() {
        return rpy_;
    }

    virtual Vec3f GetImuAcc() {
        return acc_;
    }

    virtual Vec3f GetImuOmega() {
        return omega_body_;
    }

    virtual VecXf GetContactForce() {
        return VecXf::Zero(4);
    }

    virtual void ResetJointError() {
        auto msg = drdds::msg::JointsDataCmd();

        for (int j = 0; j < 4; ++j) {
            for (int i = 0; i < dof_num_; ++i) {
                msg.data.joints_data[i].position = 0;
                msg.data.joints_data[i].velocity = 0;
                msg.data.joints_data[i].torque = 0;
                msg.data.joints_data[i].kp = 0;
                msg.data.joints_data[i].kd = 0;
                msg.data.joints_data[i].control_word = control_word[j];
            }
            std::cout << "ResetJointError publish!! \n" << std::endl;
            joint_cmd_pub_->publish(msg);
            usleep(10 * 1000);
        }
    }

    virtual void SetJointCommand(Eigen::Matrix<float, Eigen::Dynamic, 5> input) {
        joint_cmd_ = input;
        auto msg = drdds::msg::JointsDataCmd();

        for (int i = 0; i < dof_num_; ++i) {
            msg.data.joints_data[i].position =
                    (input(i, 1) - joint_config_[i].offset) * joint_config_[i].dir;
            msg.data.joints_data[i].velocity = input(i, 3) * joint_dir_[i];
            msg.data.joints_data[i].torque = input(i, 4) * joint_dir_[i];
            msg.data.joints_data[i].kp = input(i, 0);
            msg.data.joints_data[i].kd = input(i, 2);
            msg.data.joints_data[i].control_word = kIndexMotorControl;
        }
        std::cout << "SetJointCommand publish!! \n" << std::endl;
        joint_cmd_pub_->publish(msg);
    }

    virtual VecXf GetMotorTemperture() {
        return motor_temperture_;
    }

    virtual VecXf GetDriverTemperture() {
        return driver_temperture_;
    }

    virtual double GetImuTimestamp() {
        return imu_ts_;
    }

    virtual std::vector<uint16_t> GetBatteryData() {
        return battery_data_;
    }

    virtual std::vector<uint16_t> GetDriverStatusWord() {
        return driver_status_;
    }

    virtual std::vector<uint16_t> GetJointDataID() {
        return joint_data_id_;
    }

    virtual void RefreshRobotData() {
    }

    virtual void Handler(const drdds::msg::JointsData::SharedPtr msg) {
        ++run_cnt_;
        for (int i = 0; i < dof_num_; ++i) {
            joint_pos_(i) = msg->data.joints_data[i].position * joint_dir_[i] + pos_offset_[i] / 180. * M_PI;
            joint_vel_(i) = msg->data.joints_data[i].velocity * joint_dir_[i];
            joint_tau_(i) = msg->data.joints_data[i].torque * joint_dir_[i];
            motor_temperture_(i) = float(msg->data.joints_data[i].motion_temp);
            driver_temperture_(i) = float(msg->data.joints_data[i].driver_temp);
            driver_status_[i] = msg->data.joints_data[i].status_word;
            joint_data_id_[i] = uint16_t(run_cnt_);
        }
        ri_ts_ = rclcpp::Time(msg->header.stamp).seconds();
    }

    void HandlerIMU(const drdds::msg::ImuData::SharedPtr msg) {
        rpy_ = Vec3f(Deg2Rad(msg->data.roll), Deg2Rad(msg->data.pitch), Deg2Rad(msg->data.yaw));
        acc_ << msg->data.acc_x, msg->data.acc_y, msg->data.acc_z;
        omega_body_ << msg->data.omega_x, msg->data.omega_y, msg->data.omega_z;
        imu_ts_ = rclcpp::Time(msg->header.stamp).seconds();
    }

    void HandlerHealth(const drdds::msg::BatteryData::SharedPtr msg) {
        battery_data_[0] = uint8_t(msg->data[0].battery_level);
        remain_bat = battery_data_[0] / 100;
        battery_info_[0].battery_level = msg->data[0].battery_level;
    }
};
