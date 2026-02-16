/**
 * @file quadruped_state_machine.hpp
 * @brief for robot to switch control state by user command input
 * @author Haokai Dai
 * @version 1.0
 * @date 2025-12-25
 * 
 * @copyright Copyright (c) 2025  DeepRobotics
 * 
 */
#pragma once

#include "state_base.h"
#include "state_machine_base.h"
#include "quadruped/idle_state.hpp"
#include "quadruped/standup_state.hpp"
#include "quadruped/joint_damping_state.hpp"
#include "quadruped/rl_control_state.hpp"
#include "keyboard_interface.hpp"
#include "retroid_gamepad_interface.hpp"
#include "hardware/lite3_interface.hpp"

namespace q{
class QStateMachine : public StateMachineBase{
private:

    std::shared_ptr<StateBase> idle_controller_;
    std::shared_ptr<StateBase> standup_controller_;
    std::shared_ptr<StateBase> rl_controller_;
    std::shared_ptr<StateBase> joint_damping_controller_;
    // std::shared_ptr<StateBase> car_move_controller_;

public:
    const RobotName robot_name_;
    const RemoteCommandType remote_cmd_type_;

    QStateMachine(RobotName robot_name, RemoteCommandType rct):StateMachineBase(RobotType::Quadruped),
    robot_name_(robot_name),
    remote_cmd_type_(rct) {}
    ~QStateMachine(){}

    void Start(){
        // 先创建 RobotInterface，这样 RetroidGamepadInterface 可以使用它的节点
        if(robot_name_ == RobotName::Lite3){
            ri_ptr_ = std::make_shared<Lite3Interface>("Lite3");
            cp_ptr_ = std::make_shared<ControlParameters>(robot_name_);
        }

        // 创建用户命令接口
        if(remote_cmd_type_ == RemoteCommandType::kKeyBoard){
            uc_ptr_ = std::make_shared<KeyboardInterface>(robot_name_);
        }else if(remote_cmd_type_ == RemoteCommandType::kRetroidGamepad){
            // 使用 RobotInterface 的节点创建 RetroidGamepadInterface
            if(!ri_ptr_) {
                std::cerr << "error: RobotInterface must be created before RetroidGamepadInterface!" << std::endl;
                exit(1);
            }
            uc_ptr_ = std::make_shared<RetroidGamepadInterface>(robot_name_, ri_ptr_->get_node());
        }else{
            std::cerr << "error user command interface! " << std::endl;
            exit(0);
        }
        uc_ptr_->SetMotionStateFeedback(&StateBase::msfb_);

        std::shared_ptr<ControllerData> data_ptr = std::make_shared<ControllerData>();
        data_ptr->ri_ptr = ri_ptr_;
        data_ptr->uc_ptr = uc_ptr_;
        data_ptr->cp_ptr = cp_ptr_;

        sc_ptr_ = std::make_shared<SafeController>(Quadruped, "");
        sc_ptr_->SetRobotDataSource(ri_ptr_);
        sc_ptr_->SetUserCommandDataSource(uc_ptr_);

        idle_controller_ = std::make_shared<IdleState>(robot_name_, "idle_state", data_ptr);
        standup_controller_ = std::make_shared<StandUpState>(robot_name_, "standup_state", data_ptr);
        rl_controller_ = std::make_shared<RLControlState>(robot_name_, "rl_control", data_ptr);
        joint_damping_controller_ = std::make_shared<JointDampingState>(robot_name_, "joint_damping", data_ptr);

        current_controller_ = idle_controller_;
        current_state_name_ = kIdle;
        next_state_name_ = kIdle;

        ri_ptr_->Start();
        uc_ptr_->Start();
        sc_ptr_->Start();
        current_controller_->OnEnter();
    }


    std::shared_ptr<StateBase> GetStateControllerPtr(StateName state_name){
        switch(state_name){
            case StateName::kInvalid:{
                return nullptr;
            }
            case StateName::kIdle:{
                return idle_controller_;
            }
            case StateName::kStandUp:{
                return standup_controller_;
            }
            case StateName::kRLControl:{
                return rl_controller_;
            }
            case StateName::kJointDamping:{
                return joint_damping_controller_;
            }
            default:{
                std::cerr << "error state name" << std::endl;
                return joint_damping_controller_;
            }
        }
        return nullptr;
    }

    void Stop(){
        sc_ptr_->Stop();
        uc_ptr_->Stop();
        ri_ptr_->Stop();
    }

};
};
