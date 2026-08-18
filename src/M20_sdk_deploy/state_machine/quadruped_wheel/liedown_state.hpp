/**
 * @file liedown_state.hpp
 * @brief liedown state
 * @author DeepRobotics
 * @version 1.0
 * @date 2026-02-12
 * 
 * @copyright Copyright (c) 2025  DeepRobotics
 * 
 */
#pragma once

#include "state_base.h"

namespace qw{
class LieDownState : public StateBase{
private:
    VecXf init_joint_pos_, init_joint_vel_, current_joint_pos_, current_joint_vel_;
    double time_stamp_record_, run_time_;
    VecXf goal_joint_pos_, kp_, kd_;
    MatXf joint_cmd_;
    float liedown_duration_ = 2.;

    const float init_hipx_pos_ = Deg2Rad(0.);
    const float set_wheel_kd_ = 1.;

    void GetRobotJointValue(){
        current_joint_pos_ = ri_ptr_->GetJointPosition();
        current_joint_vel_ = ri_ptr_->GetJointVelocity();
        run_time_ = ri_ptr_->GetInterfaceTimeStamp();
    }

    void RecordJointData(){
        init_joint_pos_ = current_joint_pos_;
        init_joint_vel_ = current_joint_vel_;
        time_stamp_record_ = run_time_;
    }
    
    float GetHipYPosByHeight(float h){
        float l1 = cp_ptr_->thigh_len_;
        float l2 = cp_ptr_->shank_len_;
        float default_pos = (cp_ptr_->fl_joint_lower_(1)+cp_ptr_->fl_joint_upper_(1)) / 2.;
        if(fabs(h) >= l1 + l2) {
            std::cerr << "error height input" << std::endl;
            return 0;
        }
        float theta = -acos((l1*l1+h*h-l2*l2)/(2.*h*l1));
        LimitNumber(theta, cp_ptr_->fl_joint_lower_(1), cp_ptr_->fl_joint_upper_(1));
        return theta;
    }

    float GetKneePosByHeight(float h){
        float l1 = cp_ptr_->thigh_len_;
        float l2 = cp_ptr_->shank_len_;
        float default_pos = (cp_ptr_->fl_joint_lower_(2)+cp_ptr_->fl_joint_upper_(2)) / 2.;
        if(fabs(h) >= l1 + l2) {
            std::cerr << "error height input" << std::endl;
            return 0;
        }
        float theta = M_PI-acos((l1*l1+l2*l2-h*h)/(2*l1*l2));
        LimitNumber(theta, cp_ptr_->fl_joint_lower_(2), cp_ptr_->fl_joint_upper_(2));
        return theta;
    }

public:
    LieDownState(const RobotName& robot_name, const std::string& state_name, 
        std::shared_ptr<ControllerData> data_ptr):StateBase(robot_name, state_name, data_ptr){
            init_joint_pos_ = VecXf::Zero(16); 
            init_joint_vel_ = VecXf::Zero(16);
            current_joint_pos_ = VecXf::Zero(16);
            current_joint_vel_ = VecXf::Zero(16);
            
            goal_joint_pos_ = Vec4f(init_hipx_pos_, GetHipYPosByHeight(0.03), GetKneePosByHeight(0.03), 0).replicate(4, 1);
            goal_joint_pos_(4) = -init_hipx_pos_; goal_joint_pos_(12) = -init_hipx_pos_;

            goal_joint_pos_(9) = -goal_joint_pos_(9);
            goal_joint_pos_(10) = -goal_joint_pos_(10);
            goal_joint_pos_(13) = -goal_joint_pos_(13);
            goal_joint_pos_(14) = -goal_joint_pos_(14);

            Vec4f one_leg_kp, one_leg_kd;

            one_leg_kp << 300, 300, 300, 0;
            one_leg_kd << 4.5, 4.5, 4.5, 3; //2; //0;
            kp_ = one_leg_kp.replicate(4, 1);
            kd_ = one_leg_kd.replicate(4, 1);
            liedown_duration_ = cp_ptr_->liedown_duration_;
        }
    ~LieDownState(){}

    virtual void OnEnter() {
        joint_cmd_ = MatXf::Zero(16, 5);
        joint_cmd_.col(0) = kp_;
        joint_cmd_.col(2) = kd_;
        GetRobotJointValue();
        RecordJointData();  
        StateBase::msfb_.UpdateCurrentState(RobotMotionState::LieDown);
    };

    virtual void OnExit() {
    }

    virtual void Run() {
        GetRobotJointValue();
        VecXf planning_joint_pos(current_joint_pos_.rows());
        VecXf planning_joint_vel(current_joint_pos_.rows());
        if(run_time_ - time_stamp_record_ <= liedown_duration_){
            for(int i=0;i<current_joint_pos_.rows();++i){
                planning_joint_pos(i) = GetCubicSplinePos(init_joint_pos_(i), init_joint_vel_(i), goal_joint_pos_(i), 0, 
                                                run_time_ - time_stamp_record_, liedown_duration_);
                planning_joint_vel(i) = GetCubicSplineVel(init_joint_pos_(i), init_joint_vel_(i), goal_joint_pos_(i), 0, 
                                                run_time_ - time_stamp_record_, liedown_duration_);
                if(i%4==3){
                    planning_joint_vel(i) = 0.0;
                }
            }

            joint_cmd_.col(1) = planning_joint_pos;
            joint_cmd_.col(3) = planning_joint_vel;
            joint_cmd_.col(2) = kd_;
            ri_ptr_->SetJointCommand(joint_cmd_);
        } else if (run_time_ - time_stamp_record_ <=  2.0 * liedown_duration_){
            joint_cmd_ = MatXf::Zero(16, 5);
            joint_cmd_.col(2) = kd_;
            
            ri_ptr_->SetJointCommand(joint_cmd_);
        } else {
            joint_cmd_ = MatXf::Zero(16, 5);
            ri_ptr_->SetJointCommand(joint_cmd_);
        }
    }
    virtual bool LoseControlJudge() {
        if (uc_ptr_->GetUserCommand()->target_mode == uint8_t(RobotMotionState::JointDamping)) return true;
        return false;
    }
    virtual StateName GetNextStateName() {
        if(uc_ptr_->GetUserCommand()->safe_control_mode!=0){
            return StateName::kJointDamping;
        } 
        
        if(run_time_ - time_stamp_record_ <= 2.*liedown_duration_){
            return StateName::kLieDown;
        }else{
            if(uc_ptr_->GetUserCommand()->target_mode == uint8_t(RobotMotionState::StandingUp)){
                return StateName::kStandUp;
            }
        }
        return StateName::kLieDown;
    }
};

};
