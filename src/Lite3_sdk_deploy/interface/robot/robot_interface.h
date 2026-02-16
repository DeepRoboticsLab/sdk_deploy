/**
 * @file robot_interface.h
 * @brief this file is for applying robot's control interface
 * @author DeepRobotics
 * @version 1.0
 * @date 2025-11-07
 * 
 * @copyright Copyright (c) 2025  DeepRobotics
 * 
 */
#pragma once

#include "common_types.h"
#include <atomic>
#include "rclcpp/rclcpp.hpp"
using namespace types;

namespace interface{
const int BATTERY_DATA_SIZE=8;

class RobotInterface{
private:
    /* data */
public:
    /**
     * @brief Construct a new Robot Interface object
     * @param  robot_name       robot name
     * @param  dof_num          The number of degrees of freedom of the robot.
     */
    RobotInterface(const std::string& robot_name, int dof_num):robot_name_(robot_name), dof_num_(dof_num){
        start_flag_ = true;
        joint_cmd_ = Eigen::Matrix<float, Eigen::Dynamic, 5>::Zero(dof_num_, 5);
        node_ = std::make_shared<rclcpp::Node>("rl_deploy");

    }
    virtual ~RobotInterface(){};

    std::string robot_name_;
    const int dof_num_;
    Eigen::Matrix<float, Eigen::Dynamic, 5> joint_cmd_;
    std::atomic<bool> start_flag_;
    rclcpp::Node::SharedPtr node_;

    rclcpp::Node::SharedPtr get_node() {
        return node_;
    }
    /**
     * @brief Start to get robot's state and send control command
     */
    virtual void Start() = 0;

    /**
     * @brief Stop to control the robot
     */
    virtual void Stop() = 0;

    /**
     * @brief Get the time stamp of the robot
     * @return double        time stamp
     */
    virtual double GetInterfaceTimeStamp() = 0;

    /**
     * @brief Get the joint position of the robot
     * @return VecXf        joint position vector
     */
    virtual VecXf GetJointPosition() = 0;

    /**
     * @brief Get the joint velocity of the robot
     * @return VecXf        joint velocity vector
     */
    virtual VecXf GetJointVelocity() = 0;

    /**
     * @brief Get the joint torque of the robot
     * @return VecXf        joint torque vector
     */
    virtual VecXf GetJointTorque() = 0;

    /**
     * @brief Get the roll-pitch-yaw angle of the robot base
     * @return Vec3f        roll-pitch-yaw(unit rad)
     */
    virtual Vec3f GetImuRpy() = 0;

    /**
     * @brief Get the accleration of the robot base
     * @return Vec3f        accleration(unit m*s^-2)
     */
    virtual Vec3f GetImuAcc() = 0;

    /**
     * @brief Get the angular velocity of robot base
     * @return Vec3f        angular velocity in body coordinate(unit rad/s)
     */
    virtual Vec3f GetImuOmega() = 0;

    /**
     * @brief Set the joint command in standard form
     *                  torque = kp * (qDes - q) + kd * (vDes - v) + tff
     * @param  input       a dof_num*5 matrix and each column represent kp, goal_angle_pos, kd, goal_vel, torque_feedforward
     */
    virtual void SetJointCommand(Eigen::Matrix<float, Eigen::Dynamic, 5> input) = 0;

    virtual MatXf GetJointCommand(){
        return joint_cmd_;
    }

    /**
     * @brief Get the contact force if robot have any force sensor
     * @return VecXf        force vector
     */
    virtual VecXf GetContactForce() = 0;


    /**
     * @brief Get the motor temperture 
     * @return VecXf        motor temperture
     */
    virtual VecXf GetMotorTemperture(){
        return 30.*VecXf::Ones(dof_num_);
    }

    /**
     * @brief Get the driver temperture 
     * @return VecXf        driver temperture
     */
    virtual VecXf GetDriverTemperture(){
        return VecXf::Zero(dof_num_);
    }

    /**
     * @brief Get the Battery Data
     * @return std::vector<uint16_t> level(0~100), voltage(mV), current(mA), cycles, temperture*4(C) 
     */
    virtual std::vector<uint16_t> GetBatteryData(){
        return std::vector<uint16_t>(BATTERY_DATA_SIZE, 0);
    }

    /**
     * @brief Get the Imu Timestamp 
     * @return float 
     */
    virtual double GetImuTimestamp(){
        return 0.;
    }

    /**
     * @brief Get the Driver Status Word 
     * @return std::vector<uint16_t> 
     */
    virtual std::vector<uint16_t> GetDriverStatusWord(){
        return std::vector<uint16_t>(dof_num_, 1);
    }

    /**
     * @brief Get the Joint Data ID object
     * @return std::vector<uint16_t> 
     */
    virtual std::vector<uint16_t> GetJointDataID(){
        return std::vector<uint16_t>(dof_num_, 0);
    }

    virtual void RefreshRobotData(){
    }
};


};
