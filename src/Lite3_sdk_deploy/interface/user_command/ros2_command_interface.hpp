#pragma once

// ROS 2 command input and status publication for the Lite3 SDK.
#include "user_command_interface.h"
#include "robot_interface.h"
#include "drdds/msg/robot_status.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <stdexcept>

namespace sdk_control {
using Clock = std::chrono::steady_clock;

inline std::string StateLabel(uint8_t state) {
    switch (state) {
    case types::WaitingForStand: return "idle";
    case types::StandingUp: return "stand";
    case types::JointDamping: return "damping";
    case types::LieDown: return "lie_down";
    case types::RLControlMode: return "rl_control";
    default: return "unknown";
    }
}

class Ros2CommandInterface : public interface::UserCommandInterface {
public:
    Ros2CommandInterface(types::RobotName robot, const rclcpp::Node::SharedPtr& node)
        : UserCommandInterface(robot), node_(node) {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.read_only = true;
        timeout_ = node_->declare_parameter<double>("command_timeout", 0.25, descriptor);
        max_forward_ = node_->declare_parameter<double>("max_forward_velocity", 0.7, descriptor);
        max_side_ = node_->declare_parameter<double>("max_side_velocity", 0.5, descriptor);
        max_yaw_ = node_->declare_parameter<double>("max_yaw_velocity", 0.7, descriptor);
        for (double value : {timeout_, max_forward_, max_side_, max_yaw_})
            if (!std::isfinite(value) || value < 0.)
                throw std::invalid_argument("Command timeout/limits must be finite and nonnegative");
        if (timeout_ <= 0.) throw std::invalid_argument("command_timeout must be positive");

        // Volatile, depth one: do not replay a latched command at startup.
        velocity_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
            "~/cmd_vel", rclcpp::QoS(1).best_effort().durability_volatile(),
            [this](geometry_msgs::msg::Twist::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                if (!running_) return;
                if (!std::isfinite(msg->linear.x) || !std::isfinite(msg->linear.y) ||
                    !std::isfinite(msg->angular.z) || msg->linear.z != 0. ||
                    msg->angular.x != 0. || msg->angular.y != 0.) {
                    ++rejected_; velocity_ = {}; have_velocity_ = false; return;
                }
                velocity_ = *msg;
                velocity_.linear.x = std::clamp(msg->linear.x, -max_forward_, max_forward_);
                velocity_.linear.y = std::clamp(msg->linear.y, -max_side_, max_side_);
                velocity_.angular.z = std::clamp(msg->angular.z, -max_yaw_, max_yaw_);
                velocity_time_ = Clock::now();
                have_velocity_ = true;
            });
        state_sub_ = node_->create_subscription<std_msgs::msg::String>(
            "~/command_state", rclcpp::QoS(1).reliable().durability_volatile(),
            [this](std_msgs::msg::String::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                if (!running_) return;
                int target = -1;
                if (msg->data == "stand") target = types::StandingUp;
                if (msg->data == "lie_down") target = types::LieDown;
                if (msg->data == "rl_control") target = types::RLControlMode;
                if (msg->data == "damping") target = types::JointDamping;
                if (target < 0) { ++rejected_; return; }
                // A damping request cannot be overwritten before the next control tick.
                if (pending_state_ != types::JointDamping) pending_state_ = target;
                state_time_ = Clock::now();
            });
    }

    void Start() override {
        std::lock_guard<std::mutex> lock(mutex_);
        running_ = true;
        have_velocity_ = false;
        pending_state_ = -1;
    }
    void Stop() override {
        std::lock_guard<std::mutex> lock(mutex_);
        running_ = false;
        ZeroVelocity();
    }
    types::UserCommand* GetUserCommand() override {
        // Existing consumers use pointers. Each reader gets an independent snapshot.
        thread_local types::UserCommand snapshot;
        std::lock_guard<std::mutex> lock(mutex_);
        snapshot = *usr_cmd_;
        return &snapshot;
    }
    void SetSafetyMode(uint8_t mode) override {
        std::lock_guard<std::mutex> lock(mutex_);
        usr_cmd_->safe_control_mode = mode;
        if (mode) { pending_state_ = -1; have_velocity_ = false; ZeroVelocity(); }
    }
    void SetTargetMode(uint8_t mode) override {
        std::lock_guard<std::mutex> lock(mutex_);
        usr_cmd_->target_mode = mode;
        pending_state_ = -1;
        have_velocity_ = false;
        ZeroVelocity();
    }

    // Called by the FSM at 50 Hz, before it reads commands. State guards still
    // belong to the existing controllers (including stand/lie trajectory timing).
    void Update(uint8_t current_state) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!running_) return;
        const auto now = Clock::now();
        if (current_state == types::JointDamping && last_state_ != current_state) {
            pending_state_ = -1;
            usr_cmd_->target_mode = types::JointDamping;
            have_velocity_ = false;
        }
        if (last_state_ == types::JointDamping && current_state == types::WaitingForStand)
            usr_cmd_->target_mode = types::WaitingForStand;
        last_state_ = current_state;
        if (pending_state_ >= 0) {
            const bool fresh = std::chrono::duration<double>(now - state_time_).count() <= timeout_;
            const bool allowed = pending_state_ == types::JointDamping ||
                pending_state_ == current_state ||
                (pending_state_ == types::StandingUp &&
                    (current_state == types::WaitingForStand || current_state == types::LieDown)) ||
                (pending_state_ == types::LieDown &&
                    (current_state == types::StandingUp || current_state == types::RLControlMode)) ||
                (pending_state_ == types::RLControlMode && current_state == types::StandingUp);
            if (fresh && allowed && usr_cmd_->safe_control_mode == 0) {
                usr_cmd_->target_mode = pending_state_;
                if (pending_state_ != types::RLControlMode) have_velocity_ = false;
            } else { ++rejected_; }
            pending_state_ = -1;
        }
        timed_out_ = !have_velocity_ ||
            std::chrono::duration<double>(now - velocity_time_).count() > timeout_;
        ZeroVelocity();
        if (!timed_out_ && current_state == types::RLControlMode &&
            usr_cmd_->target_mode == types::RLControlMode && usr_cmd_->safe_control_mode == 0) {
            usr_cmd_->forward_vel_scale = velocity_.linear.x;
            usr_cmd_->side_vel_scale = velocity_.linear.y;
            usr_cmd_->turnning_vel_scale = velocity_.angular.z;
        }
        usr_cmd_->time_stamp = std::chrono::duration<double>(now.time_since_epoch()).count();
    }

    void GetDiagnostics(bool& timed_out, uint64_t& rejected) {
        std::lock_guard<std::mutex> lock(mutex_);
        timed_out = timed_out_; rejected = rejected_;
    }

private:
    void ZeroVelocity() {
        usr_cmd_->forward_vel_scale = 0.f;
        usr_cmd_->side_vel_scale = 0.f;
        usr_cmd_->turnning_vel_scale = 0.f;
    }
    rclcpp::Node::SharedPtr node_;
    std::mutex mutex_;
    bool running_{false}, have_velocity_{false}, timed_out_{true};
    uint8_t last_state_{types::WaitingForStand};
    int pending_state_{-1};
    uint64_t rejected_{0};
    double timeout_, max_forward_, max_side_, max_yaw_;
    Clock::time_point velocity_time_{}, state_time_{};
    geometry_msgs::msg::Twist velocity_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
};

class Ros2ControlStatus {
public:
    Ros2ControlStatus(const std::shared_ptr<interface::RobotInterface>& robot,
                      const std::shared_ptr<interface::UserCommandInterface>& command,
                      const std::string& source)
        : robot_(robot), command_(command), source_(source),
          ros_command_(std::dynamic_pointer_cast<Ros2CommandInterface>(command)) {
        status_pub_ = robot_->get_node()->create_publisher<drdds::msg::RobotStatus>(
            "~/status", 1);
        command_pub_ = robot_->get_node()->create_publisher<geometry_msgs::msg::TwistStamped>(
            "~/effective_cmd_vel", 1);
    }

    void UpdateCommand(uint8_t state) {
        auto now = Clock::now();
        if (now < next_command_) return;
        Advance(next_command_, now, std::chrono::milliseconds(20));
        if (ros_command_) ros_command_->Update(state);
        ++updates_;
        geometry_msgs::msg::TwistStamped msg;
        msg.header.stamp = robot_->get_node()->now();
        msg.header.frame_id = "base_link";
        msg.twist = Velocity(*command_->GetUserCommand());
        command_pub_->publish(msg);
    }

    void Publish(uint8_t state) {
        const auto now = Clock::now();
        if (now < next_status_) return;
        Advance(next_status_, now, std::chrono::milliseconds(100));
        drdds::msg::RobotStatus msg;
        msg.header.stamp = robot_->get_node()->now();
        msg.header.frame_id = "base_link";
        msg.robot = robot_->robot_name_;
        msg.command_source = source_;
        msg.current_state_id = state;
        msg.current_state = StateLabel(state);
        const auto cmd = *command_->GetUserCommand();
        msg.requested_state_id = cmd.target_mode;
        msg.requested_state = StateLabel(cmd.target_mode);
        msg.safety_mode = cmd.safe_control_mode;
        msg.command = Velocity(cmd);
        msg.command_updates = updates_;
        if (ros_command_) ros_command_->GetDiagnostics(msg.command_timed_out, msg.rejected_commands);
        msg.joint_stamp = robot_->GetInterfaceTimeStamp();
        msg.imu_stamp = robot_->GetImuTimestamp();
        if (msg.joint_stamp != last_joint_stamp_) joint_time_ = now;
        if (msg.imu_stamp != last_imu_stamp_) imu_time_ = now;
        last_joint_stamp_ = msg.joint_stamp;
        last_imu_stamp_ = msg.imu_stamp;
        msg.telemetry_fresh = msg.joint_stamp > 0. && msg.imu_stamp > 0. &&
            now - joint_time_ < std::chrono::milliseconds(250) &&
            now - imu_time_ < std::chrono::milliseconds(250);
        const auto pos = robot_->GetJointPosition();
        const auto vel = robot_->GetJointVelocity();
        const auto tau = robot_->GetJointTorque();
        msg.joints.header = msg.header;
        const bool wheeled = robot_->dof_num_ == 16;
        for (const auto& leg : {"fl", "fr", "hl", "hr"}) {
            for (const auto& joint : {"hipx", "hipy", "knee"})
                msg.joints.name.push_back(std::string(leg) + "_" + joint + "_joint");
            if (wheeled) msg.joints.name.push_back(std::string(leg) + "_wheel_joint");
        }
        msg.joints.position.assign(pos.data(), pos.data() + pos.size());
        msg.joints.velocity.assign(vel.data(), vel.data() + vel.size());
        msg.joints.effort.assign(tau.data(), tau.data() + tau.size());
        const auto rpy = robot_->GetImuRpy();
        const auto omega = robot_->GetImuOmega();
        msg.base_rpy.x = rpy.x(); msg.base_rpy.y = rpy.y(); msg.base_rpy.z = rpy.z();
        msg.base_angular_velocity.x = omega.x();
        msg.base_angular_velocity.y = omega.y();
        msg.base_angular_velocity.z = omega.z();
        status_pub_->publish(msg);
    }

private:
    static geometry_msgs::msg::Twist Velocity(const types::UserCommand& cmd) {
        geometry_msgs::msg::Twist result;
        result.linear.x = cmd.forward_vel_scale;
        result.linear.y = cmd.side_vel_scale;
        result.angular.z = cmd.turnning_vel_scale;
        return result;
    }
    static void Advance(Clock::time_point& deadline, Clock::time_point now,
                        std::chrono::milliseconds period) {
        if (deadline == Clock::time_point{}) deadline = now;
        do { deadline += period; } while (deadline <= now);
    }
    std::shared_ptr<interface::RobotInterface> robot_;
    std::shared_ptr<interface::UserCommandInterface> command_;
    std::string source_;
    std::shared_ptr<Ros2CommandInterface> ros_command_;
    uint64_t updates_{0};
    double last_joint_stamp_{0.}, last_imu_stamp_{0.};
    Clock::time_point next_command_{}, next_status_{}, joint_time_{}, imu_time_{};
    rclcpp::Publisher<drdds::msg::RobotStatus>::SharedPtr status_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr command_pub_;
};
} // namespace sdk_control
