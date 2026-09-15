/**
 * @file joint_damping_state.hpp
 * @brief joint passive control state
 * @author DeepRobotics
 * @version 1.0
 * @date 2025-11-07
 * 
 * @copyright Copyright (c) 2025  DeepRobotics
 * 
 */
#pragma once

#include "state_base.h"

namespace q{
class JointDampingState : public StateBase{
private:
    double time_record_, run_time_;
    MatXf joint_cmd_;
public:
    JointDampingState(const RobotName& robot_name, const std::string& state_name, 
        std::shared_ptr<ControllerData> data_ptr):StateBase(robot_name, state_name, data_ptr){
            Vec3f one_leg_kd;
            one_leg_kd << cp_ptr_->swing_leg_kd_;
            VecXf kd_ = one_leg_kd.replicate(4, 1);
            joint_cmd_ = MatXf::Zero(12, 5);
            joint_cmd_.col(2) = kd_;
        }
    ~JointDampingState(){}

    virtual void OnEnter() {
        time_record_ = ri_ptr_->GetInterfaceTimeStamp();
        run_time_ = ri_ptr_->GetInterfaceTimeStamp();
        msfb_.UpdateCurrentState(RobotMotionState::JointDamping);
    };
    virtual void OnExit() {
        // 转出 JointDamping 时重置 target_mode，避免残留指令影响后续状态判断
        uc_ptr_->SetTargetMode(uint8_t(RobotMotionState::WaitingForStand));
        std::cout << "Joint Damping complete, switching to Idle. Press Y/Z to stand up." << std::endl;
    }
    virtual void Run() {
        run_time_ = ri_ptr_->GetInterfaceTimeStamp();
        ri_ptr_->SetJointCommand(joint_cmd_);
    }
    virtual bool LoseControlJudge() {
        return false;
    }
    virtual StateName GetNextStateName() {
        if(run_time_ - time_record_ < 3.){
            return StateName::kJointDamping;
        }
        return StateName::kIdle;
    }
};


};

