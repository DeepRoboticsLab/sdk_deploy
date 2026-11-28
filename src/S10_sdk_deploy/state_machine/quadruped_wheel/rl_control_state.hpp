/**
 * @file rl_control_state.hpp
 * @brief rl policy runnning state for quadruped-wheel robot
 * @author DeepRobotics
 * @version 1.0
 * @date 2025-11-07
 * 
 * @copyright Copyright (c) 2025  DeepRobotics
 * 
 */
#pragma once
#include "state_base.h"
#include "policy_runner_base.hpp"
#include "s10_policy_runner.hpp"
#include "robot_interface.h"
#include "user_command_interface.h"
#include "basic_function.hpp"

namespace qw {
    class RLControlState : public StateBase {
    private:
        RobotBasicState rbs_[2];
        std::atomic<int> rbs_write_index_{0};
        int getrbsReadIndex() const { return 1 - rbs_write_index_.load(std::memory_order_acquire); }

        int state_run_cnt_;

        std::shared_ptr<PolicyRunnerBase> policy_ptr_;
        std::shared_ptr<S10PolicyRunner> s10_policy_;

        std::thread run_policy_thread_;
        bool start_flag_ = true;

        float policy_cost_time_ = 1;

        Eigen::MatrixXf acc_rot = Eigen::MatrixXf::Zero(20, 3);
        int acc_rot_count = 0;

        void UpdateRobotObservation();
        void PolicyRunner();

    public:
        RLControlState(const RobotName &robot_name, const std::string &state_name,
                       std::shared_ptr<ControllerData> data_ptr);
        ~RLControlState();

        virtual void OnEnter();
        virtual void OnExit();
        virtual void Run();
        virtual bool LoseControlJudge();
        bool PostureUnsafeCheck();
        virtual StateName GetNextStateName();
    };
};