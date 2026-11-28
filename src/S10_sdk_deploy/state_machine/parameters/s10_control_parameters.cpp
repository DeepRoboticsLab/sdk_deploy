#include "control_parameters.h"
#include "quadruped_wheel/rl_control_state.hpp"

#include <filesystem>
#include <onnxruntime_c_api.h>
#include <onnxruntime_cxx_api.h>

void ControlParameters::GenerateS10Parameters(){
    body_len_x_ = 0.225 * 2;
    body_len_y_ = 0.05254 * 2;
    hip_len_ = 0.05;
    thigh_len_ = 0.18;
    shank_len_ = 0.18;
    pre_height_ = 0.08;
    stand_height_ = 0.28;
    swing_leg_kp_ << 120., 120., 120.;
    swing_leg_kd_ << 2., 2., 2.;

    fl_joint_lower_ << -0.6109, -2.5307, -2.7227;
    fl_joint_upper_ << 0.6109, 2.5307, 2.7227;
    joint_vel_limit_ << 25.76, 25.76, 25.76;
    torque_limit_ << 50, 50, 50;
}

struct S10PolicyRunner::OrtImpl {
    Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "S10PolicyRunner"};
    Ort::SessionOptions session_options{};
    Ort::Session session{nullptr};
    Ort::MemoryInfo memory_info{Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)};
};

S10PolicyRunner::S10PolicyRunner(const std::string &policy_name, const std::string &policy_path)
    : PolicyRunnerBase(policy_name),
      policy_path_(policy_path),
      ort_(std::make_unique<OrtImpl>()) {
    dof_default_eigen_policy.setZero(action_dim);
    dof_default_eigen_robot.setZero(action_dim);
    dof_default_eigen_policy << 0.05, -0.35,  0.65,
                                -0.05, -0.35,  0.65,
                                0.05,  0.35, -0.65,
                                -0.05,  0.35, -0.65,
                                0.0, 0.0, 0.0, 0.0;
    dof_default_eigen_robot << 0.05, -0.35,  0.65, 0.0,
                               -0.05, -0.35,  0.65, 0.0,
                               0.05,  0.35, -0.65, 0.0,
                               -0.05,  0.35, -0.65, 0.0;
    SetDecimation(4);
    ort_->session_options.SetIntraOpNumThreads(4);
    ort_->session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

    if (access(policy_path_.c_str(), F_OK) != 0) {
        std::cerr << "Model file not found: " << policy_path_ << std::endl;
        throw std::runtime_error("Model file missing");
    }

    // 加载模型
    ort_->session = Ort::Session(ort_->env, policy_path_.c_str(), ort_->session_options);
    const S10PolicyParams& params = GetS10PolicyParams();
    kp_ = Eigen::Map<const VecXf>(params.kp.data(), motor_num);
    kd_ = Eigen::Map<const VecXf>(params.kd.data(), motor_num);

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

    ort_->memory_info = Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);
}

S10PolicyRunner::~S10PolicyRunner() = default;

std::vector<int> S10PolicyRunner::generate_permutation(
    const std::vector<std::string>& from,
    const std::vector<std::string>& to,
    int default_index) {
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

void S10PolicyRunner::OnEnter() {
    run_cnt_ = 0;
    cmd_vel_input_.setZero();
    last_action_eigen.setZero(action_dim);
    tmp_action_eigen.setZero(action_dim);
    motor_p_eigen.setZero(12);
    motor_v_eigen.setZero(motor_num);
}

VecXf S10PolicyRunner::Onnx_infer(VecXf current_observation) {
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        ort_->memory_info,
        current_observation.data(),
        current_observation.size(),
        input_observationShape.data(),
        input_observationShape.size()
    );

    std::vector<Ort::Value> inputs;
    inputs.emplace_back(std::move(input_tensor));  // 避免拷贝构造

    auto outputs = ort_->session.Run(
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

RobotAction S10PolicyRunner::getRobotAction(const RobotBasicState &ro, const UserCommand &uc) {
    Vec3f base_omgea = ro.base_omega * omega_scale_;
    Vec3f projected_gravity = ro.base_rot_mat.inverse() * gravity_direction;
    Vec3f command = Vec3f(uc.forward_vel_scale * 1.5, uc.side_vel_scale * 0.5, uc.turnning_vel_scale * 0.6);

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

    current_action_eigen = Onnx_infer(current_observation_);
    last_action_eigen = current_action_eigen;

    if (run_cnt_ < 150){
        float ratio = run_cnt_/ 150.0;
        current_action_eigen *= ratio;
        robot_action.kp = kp_ * (0.2 + 0.8 * ratio);
        robot_action.kd = kd_ * 1.5;
    }
    else{
        robot_action.kp = kp_;
        robot_action.kd = kd_;
    }

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

void S10PolicyRunner::setDefaultJointPos(const VecXf& pos) {
    dof_pos_default_.setZero(motor_num);
    for(int i=0;i<motor_num;++i) {
        dof_pos_default_(i) = pos(i);
    }
}

double S10PolicyRunner::getCurrentTime() {
    clock_gettime(1, &system_time);
    return system_time.tv_sec + system_time.tv_nsec / 1e9;
}

namespace qw {

void RLControlState::UpdateRobotObservation() {
    int write_idx = rbs_write_index_.load(std::memory_order_relaxed);
    RobotBasicState& buffer = rbs_[write_idx];

    buffer.base_rpy = ri_ptr_->GetImuRpy();
    buffer.base_rot_mat = RpyToRm(buffer.base_rpy);
    buffer.base_omega = ri_ptr_->GetImuOmega();
    buffer.base_acc = ri_ptr_->GetImuAcc();
    buffer.joint_pos = ri_ptr_->GetJointPosition();
    buffer.joint_vel = ri_ptr_->GetJointVelocity();
    buffer.joint_tau = ri_ptr_->GetJointTorque();

    // 储存
    buffer.flt_base_acc_mat.row(acc_rot_count) = buffer.base_acc.transpose();
    acc_rot_count += 1;
    acc_rot_count = acc_rot_count % 20;

    rbs_write_index_.store(1 - write_idx,  std::memory_order_release);
}

void RLControlState::PolicyRunner() {
    int run_cnt_record = -1;
    while (start_flag_) {
        if (state_run_cnt_ % policy_ptr_->decimation_ == 0 && state_run_cnt_ != run_cnt_record) {
            timespec start_timestamp, end_timestamp;
            clock_gettime(CLOCK_MONOTONIC, &start_timestamp);
            auto ra = policy_ptr_->getRobotAction(rbs_[getrbsReadIndex()], *(uc_ptr_->GetUserCommand()));

            MatXf res = ra.ConvertToMat();

            ri_ptr_->SetJointCommand(res);
            run_cnt_record = state_run_cnt_;
            clock_gettime(CLOCK_MONOTONIC, &end_timestamp);
            policy_cost_time_ = (end_timestamp.tv_sec - start_timestamp.tv_sec) * 1e3
                                + (end_timestamp.tv_nsec - start_timestamp.tv_nsec) / 1e6;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
}

RLControlState::RLControlState(const RobotName &robot_name, const std::string &state_name,
                               std::shared_ptr<ControllerData> data_ptr)
    : StateBase(robot_name, state_name, data_ptr) {
    if (robot_name_ == RobotName::S10) {
        namespace fs = std::filesystem;
        fs::path base = fs::path(__FILE__).parent_path();
        auto model_path = fs::canonical(base / ".." / ".." / "policy" / "policy.onnx");
        s10_policy_ = std::make_shared<S10PolicyRunner>("s10_policy", model_path.string());
    }

    policy_ptr_ = s10_policy_;
    if (!policy_ptr_) {
        std::cerr << "error policy" << std::endl;
        exit(0);
    }
    policy_ptr_->DisplayPolicyInfo();
}

RLControlState::~RLControlState() = default;

void RLControlState::OnEnter() {
    state_run_cnt_ = -1;
    start_flag_ = true;
    run_policy_thread_ = std::thread(std::bind(&RLControlState::PolicyRunner, this));
    policy_ptr_->OnEnter();
    StateBase::msfb_.UpdateCurrentState(RobotMotionState::RLControlMode);
}

void RLControlState::OnExit() {
    start_flag_ = false;
    run_policy_thread_.join();
    state_run_cnt_ = -1;
}

void RLControlState::Run() {
    UpdateRobotObservation();
    state_run_cnt_++;
}

bool RLControlState::LoseControlJudge() {
    if (uc_ptr_->GetUserCommand()->target_mode == uint8_t(RobotMotionState::JointDamping)) return true;
    return PostureUnsafeCheck();
}

bool RLControlState::PostureUnsafeCheck() {
    // Vec3f rpy = ri_ptr_->GetImuRpy();
    // if(rpy(0) > 30./180*M_PI || rpy(1) > 45./180*M_PI){
    //     std::cout << "posture value: " << 180./M_PI*rpy.transpose() << std::endl;
    //     return true;
    // }
    return false;
}

StateName RLControlState::GetNextStateName() {
    if (uc_ptr_->GetUserCommand()->safe_control_mode != 0)
        return StateName::kJointDamping;
    if (uc_ptr_->GetUserCommand()->target_mode == uint8_t(RobotMotionState::LieDown))
        return StateName::kLieDown;

    return StateName::kRLControl;
}

}  // namespace qw