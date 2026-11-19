/**
 * @file m20_policy_runner.hpp
 * @brief m20_policy_runner
 * @author Bo (Percy) Peng
 * @version 1.0
 * @date 2025-11-07
 * 
 * @copyright Copyright (c) 2025  DeepRobotics
 * 
 */

#pragma once
#define PI 3.14159265358979323846

#include "policy_runner_base.hpp"
#include "fault_injection.hpp"
#include "policy_monitor.hpp"
#include <atomic>
#include <ctime>
#include <cmath>
#include <utility>
#include <onnxruntime_cxx_api.h>
#include <onnxruntime_c_api.h>

class M20PolicyRunner : public PolicyRunnerBase {
private:
    VecXf kp_, kd_;
    VecXf dof_default_eigen_policy, dof_default_eigen_robot;
    Vec3f max_cmd_vel_, gravity_direction = Vec3f(0., 0., -1.);
    VecXf dof_pos_default_;
    timespec system_time;

    const int motor_num = 16;
    const int observation_dim = 57;
    const int action_dim = 16;
    float agent_timestep = 0.02;
    float current_time;
    bool is_fallen = true;

    VecXf joint_pos_rl = VecXf(action_dim);// in rl squenece
    VecXf joint_vel_rl = VecXf(action_dim);
    
    const std::string policy_path_;

    float omega_scale_ = 0.25;
    float dof_vel_scale_ = 0.05;
    VecXf imu_w_eigen, base_acc_eigen, motor_p_eigen, motor_v_eigen,
          current_action_eigen, last_action_eigen, current_observation_, projected_gravity,
          tmp_action_eigen;

    RobotAction robot_action;
    std::vector<std::string> robot_order = {
        "fl_hipx_joint", "fl_hipy_joint", "fl_knee_joint", "fl_wheel_joint",
        "fr_hipx_joint", "fr_hipy_joint", "fr_knee_joint", "fr_wheel_joint",
        "hl_hipx_joint", "hl_hipy_joint", "hl_knee_joint", "hl_wheel_joint",
        "hr_hipx_joint", "hr_hipy_joint", "hr_knee_joint", "hr_wheel_joint"};


    std::vector<std::string> policy_order = {
        "fl_hipx_joint", "fl_hipy_joint", "fl_knee_joint",
        "fr_hipx_joint", "fr_hipy_joint", "fr_knee_joint",
        "hl_hipx_joint", "hl_hipy_joint", "hl_knee_joint",
        "hr_hipx_joint", "hr_hipy_joint", "hr_knee_joint",
        "fl_wheel_joint", "fr_wheel_joint", "hl_wheel_joint", "hr_wheel_joint",
    };


    std::vector<float> action_scale_robot = {0.125, 0.25, 0.25, 5,
                                             0.125, 0.25, 0.25, 5,
                                             0.125, 0.25, 0.25, 5,
                                             0.125, 0.25, 0.25, 5};


    Ort::SessionOptions session_options_;
    Ort::Session session_{nullptr};
    
    Ort::Env env_;
    std::vector<int> robot2policy_idx, policy2robot_idx;

    const char* input_names_[1] = {"obs"}; // must keep the same as model export
    const char* output_names_[1] = {"actions"};
    VecXf command;
    Ort::MemoryInfo memory_info{nullptr};
    const std::array<int64_t, 2> input_observationShape = {1, observation_dim};
    
    float time_step = 0.;
    int stop_count = 1000;
    std::atomic<PolicyMonitor::Level> stop_level_{PolicyMonitor::kOk};

    // Clips raw policy outputs (before scaling) and decides when to stop the policy
    // (policy_monitor.hpp). Training used rsl_rl clip_actions = 100 (effectively
    // unclipped), so these bounds are ours. Limits come from SDK_ACTION_CLIP etc.
    PolicyMonitor monitor_{PolicyMonitor::Config::FromEnv()};

public:
    M20PolicyRunner(const std::string &policy_name, const std::string &policy_path) :
            PolicyRunnerBase(policy_name), policy_path_(policy_path),env_(ORT_LOGGING_LEVEL_WARNING, "M20PolicyRunner"),
            session_options_{},
            session_{nullptr},
            memory_info(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)) {

        dof_default_eigen_policy.setZero(action_dim);
        dof_default_eigen_robot.setZero(action_dim);
        dof_default_eigen_policy << 0.0, -0.3,  0.6, 
                                    0.0, -0.3,  0.6,  
                                    0.0,  0.3, -0.6,  
                                    0.0,  0.3, -0.6, 
                                    0.0, 0.0, 0.0, 0.0;
        dof_default_eigen_robot << 0.0, -0.3,  0.6, 0.0,
                                   0.0, -0.3,  0.6, 0.0,
                                   0.0,  0.3, -0.6, 0.0,
                                   0.0,  0.3, -0.6, 0.0;
        SetDecimation(4);
        session_options_.SetIntraOpNumThreads(4);
        session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
        
        if (access(policy_path_.c_str(), F_OK) != 0) {
            std::cerr << "Model file not found: " << policy_path_ << std::endl;
            throw std::runtime_error("Model file missing");
            }

        // 加载模型
        session_ = Ort::Session(env_, policy_path_.c_str(), session_options_);
        kp_ = Vec4f(80, 80, 80, 0.).replicate(4, 1);
        kd_ = Vec4f(2, 2, 2, 0.6).replicate(4, 1);
        
        robot2policy_idx = generate_permutation(robot_order, policy_order);
        policy2robot_idx = generate_permutation(policy_order, robot_order);
        // for (int i = 0; i < action_dim; ++i){
        //     std::cout << "robot2policy_idx[" << i << "]: " << robot2policy_idx[i] << std::endl;
        //     std::cout << "policy2robot_idx[" << i << "]: " << policy2robot_idx[i] << std::endl;
        // }

        robot_action.kp = kp_;
        robot_action.kd = kd_;
        robot_action.tau_ff = VecXf::Zero(motor_num);
        robot_action.goal_joint_pos = VecXf::Zero(motor_num);
        robot_action.goal_joint_vel = VecXf::Zero(motor_num);


        current_observation_.setZero(observation_dim);
        last_action_eigen.setZero(action_dim);
        tmp_action_eigen.setZero(action_dim);
        current_action_eigen.setZero(action_dim);

        memory_info = Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);
    }

    ~M20PolicyRunner() override = default;

    std::vector<int> generate_permutation(
        const std::vector<std::string>& from, 
        const std::vector<std::string>& to, 
        int default_index = 0) 
    {
        std::unordered_map<std::string, int> idx_map;
        for (int i = 0; i < from.size(); ++i) {
            idx_map[from[i]] = i;
        }

        std::vector<int> perm;
        for (const auto& name : to) {
            auto it = idx_map.find(name);
            if (it != idx_map.end()) {
                perm.push_back(it->second);
            } else {
                perm.push_back(default_index);  // 如果找不到，就填默认值
            }
        }

        return perm;
    }

    void DisplayPolicyInfo(){}

    void OnEnter() {
        run_cnt_ = 0;
        cmd_vel_input_.setZero();
        last_action_eigen.setZero(action_dim);
        tmp_action_eigen.setZero(action_dim);
        motor_p_eigen.setZero(12);
        motor_v_eigen.setZero(motor_num);
        monitor_.Reset();
        stop_level_.store(PolicyMonitor::kOk);
        const auto& c = monitor_.config();
        std::printf("[M20PolicyRunner] action clip / soft stop at |raw|: hipx %.1f/%.1f hipy %.1f/%.1f "
                    "knee %.1f/%.1f wheel %.1f/%.1f; soft stop also: %d clipped steps in a row, tilt > %.0f deg; "
                    "hard stop: tilt > %.0f deg\n",
                    c.clip[0], c.raw_limit[0], c.clip[1], c.raw_limit[1], c.clip[2], c.raw_limit[2],
                    c.clip[3], c.raw_limit[3], c.clip_steps_limit, c.soft_tilt_deg, c.hard_tilt_deg);
        std::fflush(stdout);
    }

    // Set from the policy thread, read from the FSM thread.
    PolicyMonitor::Level StopLevel() const { return stop_level_.load(); }
    std::string StopReason() const { return monitor_.reason(); }

    void PrintActionStats() const {
        std::printf("[M20PolicyRunner] %d policy steps, %d with clipping. Max |raw action| and clip count per policy joint:\n",
                    monitor_.steps(), monitor_.clipped_steps());
        for (int i = 0; i < action_dim; ++i)
            std::printf("  %-15s %7.3f %5d\n", policy_order[i].c_str(), monitor_.raw_abs_max()(i), monitor_.clip_count()(i));
        std::fflush(stdout);
    }

    VecXf Onnx_infer(VecXf current_observation){
        
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            memory_info,
            current_observation.data(),
            current_observation.size(),
            input_observationShape.data(), 
            input_observationShape.size()
        );

        std::vector<Ort::Value> inputs;
        inputs.emplace_back(std::move(input_tensor));  // 避免拷贝构造
        
        auto outputs = session_.Run(
            Ort::RunOptions{nullptr},
            input_names_,
            inputs.data(),
            1,
            output_names_,
            1
        );

        float* action_data = outputs[0].GetTensorMutableData<float>();
        Eigen::Map<Eigen::VectorXf> action_map(action_data, action_dim);
        return VecXf(action_map);  // 返回一个Eigen向量的副本
    }

    RobotAction getRobotAction(const RobotBasicState &ro, const UserCommand &uc) {

        Vec3f base_omgea = ro.base_omega * omega_scale_;
        Vec3f projected_gravity = ro.base_rot_mat.inverse() * gravity_direction;
        Vec3f command = Vec3f(uc.forward_vel_scale, uc.side_vel_scale, uc.turnning_vel_scale);

        for (int i = 0; i < action_dim; ++i){
            joint_pos_rl(i) = ro.joint_pos(robot2policy_idx[i]);
            joint_vel_rl(i) = ro.joint_vel(robot2policy_idx[i]) * dof_vel_scale_;
        }
        joint_pos_rl.segment(12, 4).setZero();

        joint_pos_rl -= dof_default_eigen_policy;
        current_observation_<<base_omgea, 
                              projected_gravity, 
                              command, 
                              joint_pos_rl, 
                              joint_vel_rl, 
                              last_action_eigen;

        // Once stopped, never run the policy again until RL control is re-entered; hold
        // the measured pose until the state machine switches to joint damping.
        float tilt = PolicyMonitor::TiltDeg(projected_gravity);
        fault::Tilt(tilt, run_cnt_, 1.f / agent_timestep);  // no-op unless SDK_FAULT_INJECTION
        if (monitor_.level() != PolicyMonitor::kOk || monitor_.CheckPosture(tilt) != PolicyMonitor::kOk)
            return HoldAction(ro);

        current_action_eigen = Onnx_infer(current_observation_);
        fault::PolicyAction(current_action_eigen, run_cnt_, 1.f / agent_timestep);  // no-op unless SDK_FAULT_INJECTION

        if (monitor_.CheckAndClip(current_action_eigen, &policy_order) != PolicyMonitor::kOk)
            return HoldAction(ro);
        // The policy sees its clipped action next step, as in training (rsl_rl clips before env.step).
        last_action_eigen = current_action_eigen;

        
        for (int i = 0; i < action_dim; ++i){
            tmp_action_eigen(i) = current_action_eigen(policy2robot_idx[i]);
            tmp_action_eigen(i) *= action_scale_robot[i];
        }
        tmp_action_eigen += dof_default_eigen_robot;
        
        for (int i = 0; i < 4; ++i){
            robot_action.goal_joint_pos.segment(i*4, 3) = tmp_action_eigen.segment(i*4, 3);
            robot_action.goal_joint_vel(i*4+3) = tmp_action_eigen(i*4+3);
        }

        
        ++run_cnt_;
        ++time_step;
        return robot_action;
    }

    // Command for a stopped policy: legs hold where they are now, wheels stop. Only sent
    // for the few milliseconds before the state machine leaves RL control.
    RobotAction HoldAction(const RobotBasicState& ro) {
        stop_level_.store(monitor_.level());
        for (int i = 0; i < 4; ++i) {
            robot_action.goal_joint_pos.segment(i * 4, 3) = ro.joint_pos.segment(i * 4, 3);
            robot_action.goal_joint_pos(i * 4 + 3) = 0.f;
            robot_action.goal_joint_vel(i * 4 + 3) = 0.f;
        }
        ++run_cnt_;
        return robot_action;
    }

    void setDefaultJointPos(const VecXf& pos){
        dof_pos_default_.setZero(motor_num); 
        for(int i=0;i<motor_num;++i) {
            dof_pos_default_(i) = pos(i);
        }
    }

    double getCurrentTime() {
        clock_gettime(1, &system_time);
        return system_time.tv_sec + system_time.tv_nsec / 1e9;
    }
};
