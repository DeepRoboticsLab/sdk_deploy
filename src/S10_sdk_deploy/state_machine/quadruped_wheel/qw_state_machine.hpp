/**
 * @file quadruped_state_machine.hpp
 * @brief for robot to switch control state by user command input
 * @author DeepRobotics
 * @version 1.0
 * @date 2025-11-07
 * 
 * @copyright Copyright (c) 2025  DeepRobotics
 * 
 */
#pragma once

#include "state_base.h"
#include "state_machine_base.h"
#include "quadruped_wheel/idle_state.hpp"
#include "quadruped_wheel/standup_state.hpp"
#include "quadruped_wheel/joint_damping_state.hpp"
#include "quadruped_wheel/rl_control_state.hpp"
#include "quadruped_wheel/liedown_state.hpp"
#include "keyboard_interface.hpp"
#include "hardware/s10_interface.hpp"
#include "udp_server.hpp"
#include "dds_command_interface.hpp"

namespace qw{
class QwStateMachine : public StateMachineBase{
private:

    std::shared_ptr<StateBase> idle_controller_;
    std::shared_ptr<StateBase> standup_controller_;
    std::shared_ptr<StateBase> rl_controller_;
    std::shared_ptr<StateBase> joint_damping_controller_;
    std::shared_ptr<StateBase> car_move_controller_;
    std::shared_ptr<StateBase> liedown_controller_;

    std::shared_ptr<UdpServer> udp_server_;

public:
    const RobotName robot_name_;
    const RemoteCommandType remote_cmd_type_;

    QwStateMachine(RobotName robot_name, RemoteCommandType rct):StateMachineBase(RobotType::QuadrupedWheel),
    robot_name_(robot_name),
    remote_cmd_type_(rct) {}
    ~QwStateMachine(){}

    void Start(){
        if(remote_cmd_type_ == RemoteCommandType::kKeyBoard){
            uc_ptr_ = std::make_shared<KeyboardInterface>(robot_name_);
        }else if(remote_cmd_type_ == RemoteCommandType::kGamepad){
            auto gp_ptr = std::make_shared<GamepadInterface>(robot_name_);
            udp_server_ = std::make_shared<UdpServer>(gp_ptr.get());
            uc_ptr_ = gp_ptr;
        }else if(remote_cmd_type_ == RemoteCommandType::kDDS){
            auto dds_ptr = std::make_shared<DdsCommandInterface>(robot_name_);
            uc_ptr_ = dds_ptr;
        }else{
            std::cerr << "error user command interface! " << std::endl;
            exit(0);
        }
        uc_ptr_->SetMotionStateFeedback(&StateBase::msfb_);

        if(robot_name_ == RobotName::S10){
   
            ri_ptr_ = std::make_shared<S10Interface>("S10");
            
            cp_ptr_ = std::make_shared<ControlParameters>(robot_name_);
        }

        // DDS 模式需要共享 ROS 2 节点
        if(remote_cmd_type_ == RemoteCommandType::kDDS){
            auto dds_ptr = std::dynamic_pointer_cast<DdsCommandInterface>(uc_ptr_);
            if(dds_ptr){
                dds_ptr->SetNode(ri_ptr_->get_node());
            }
        }

        std::shared_ptr<ControllerData> data_ptr = std::make_shared<ControllerData>();
        data_ptr->ri_ptr = ri_ptr_;
        data_ptr->uc_ptr = uc_ptr_;
        data_ptr->cp_ptr = cp_ptr_;

        sc_ptr_ = std::make_shared<SafeController>(QuadrupedWheel, "");
        sc_ptr_->SetRobotDataSource(ri_ptr_);
        sc_ptr_->SetUserCommandDataSource(uc_ptr_);

        idle_controller_ = std::make_shared<IdleState>(robot_name_, "idle_state", data_ptr);
        standup_controller_ = std::make_shared<StandUpState>(robot_name_, "standup_state", data_ptr);
        rl_controller_ = std::make_shared<RLControlState>(robot_name_, "rl_control", data_ptr);
        joint_damping_controller_ = std::make_shared<JointDampingState>(robot_name_, "joint_damping", data_ptr);
        liedown_controller_ = std::make_shared<LieDownState>(robot_name_, "liedown_state", data_ptr);

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
            case StateName::kLieDown:
            {
                return liedown_controller_;
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
