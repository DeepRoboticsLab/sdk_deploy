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
#include <memory>

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

    struct RosImpl;
    std::unique_ptr<RosImpl> ros_;


    bool IsDataUpdatedFinished() {
        bool res = data_updated_[0];
        for (int i = 0; i < dof_num_; ++i) {
            res = res & data_updated_[i];
        }
        return res;
    }

    void ResetPositionOffset();
    void PublishJointCommand(const Eigen::Matrix<float, Eigen::Dynamic, 5>& input, bool zero_kp_for_wheel);

public:
    int run_cnt_ = 0;

    DdsInterface(const std::string &robot_name, const int &dof_num);

    virtual ~DdsInterface();

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

    virtual void ResetJointError();

    virtual void SetJointCommand(Eigen::Matrix<float, Eigen::Dynamic, 5> input);

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
};
