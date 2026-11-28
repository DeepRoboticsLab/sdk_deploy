#include "common_types.h"
using namespace types;
#include "robot_interface.h"
#include "user_command_interface.h"
using namespace interface;
#include "dds_command_interface.hpp"

#include <chrono>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/utilities.hpp>

#include "drdds/msg/steer.hpp"
#include "std_msgs/msg/string.hpp"

namespace interface {

RobotInterface::RobotInterface(const std::string& robot_name, int dof_num)
    : robot_name_(robot_name), dof_num_(dof_num) {
    start_flag_ = true;
    joint_cmd_ = Eigen::Matrix<float, Eigen::Dynamic, 5>::Zero(dof_num_, 5);
    node_ = std::make_shared<rclcpp::Node>("rl_deploy");
}

RobotInterface::~RobotInterface() = default;

void RobotInterface::Spin() {
    rclcpp::spin(node_);
}

void RobotInterface::SpinSome() {
    rclcpp::spin_some(node_);
}

bool RobotInterface::Ok() {
    return rclcpp::ok();
}

}  // namespace interface

struct DdsCommandInterface::RosImpl {
    rclcpp::GenericSubscription::SharedPtr steer_sub;
    rclcpp::GenericSubscription::SharedPtr key_sub;
    rclcpp::Serialization<drdds::msg::Steer> steer_ser;
    rclcpp::Serialization<std_msgs::msg::String> key_ser;
};

DdsCommandInterface::DdsCommandInterface(RobotName robot_name)
    : UserCommandInterface(robot_name), ros_(std::make_unique<RosImpl>()) {
    std::memset(usr_cmd_, 0, sizeof(UserCommand));
}

DdsCommandInterface::~DdsCommandInterface() {
    Stop();
}

void DdsCommandInterface::Start() {
    if (!node_) {
        std::cerr << "[DDS] Error: node not set! Call SetNode() first." << std::endl;
        return;
    }

    // 订阅 /STEER 话题（轴指令），参考 basic_server 发出端
    ros_->steer_sub = node_->create_generic_subscription(
        "/STEER", "drdds/msg/Steer", 10,
        [this](std::shared_ptr<rclcpp::SerializedMessage> raw) {
            drdds::msg::Steer msg;
            ros_->steer_ser.deserialize_message(raw.get(), &msg);
            std::lock_guard<std::mutex> lock(cmd_mutex_);
            // 轴指令映射：参考 basic_server cmdHandle_Steer
            //   x → 前后(forward), y → 左右(side), yaw → 偏航(turnning)
            usr_cmd_->forward_vel_scale = msg.data.x;
            usr_cmd_->side_vel_scale = msg.data.y;
            usr_cmd_->turnning_vel_scale = msg.data.yaw;

            // 周期性打印轴指令（降频）
            static auto last_print = std::chrono::steady_clock::now();
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration<double>(now - last_print).count() >= 0.5) {
                last_print = now;
                std::cout << "[DDS] Steer X=" << usr_cmd_->forward_vel_scale
                          << " Y=" << usr_cmd_->side_vel_scale
                          << " Yaw=" << usr_cmd_->turnning_vel_scale << std::endl;
            }
        });

    // 订阅 /GAMEPAD_KEY 话题（按键），参考 basic_server 发出端
    ros_->key_sub = node_->create_generic_subscription(
        "/GAMEPAD_KEY", "std_msgs/msg/String", 10,
        [this](std::shared_ptr<rclcpp::SerializedMessage> raw) {
            std_msgs::msg::String msg;
            ros_->key_ser.deserialize_message(raw.get(), &msg);
            std::lock_guard<std::mutex> lock(cmd_mutex_);
            const std::string& keyStr = msg.data;
            std::cout << "[DDS] Key: " << keyStr << std::endl;

            if (keyMap.count(keyStr)) {
                KeyCode code = keyMap[keyStr];
                switch (code) {
                    case KeyCode::L1:
                        if (msfb_->GetCurrentState() == RobotMotionState::WaitingForStand
                            || msfb_->GetCurrentState() == RobotMotionState::LieDown) {
                            usr_cmd_->target_mode = uint8_t(RobotMotionState::StandingUp);
                            std::cout << "[DDS][MODE] Standing Up" << std::endl;
                        }
                        break;
                    case KeyCode::L2:
                        if (msfb_->GetCurrentState() == RobotMotionState::StandingUp) {
                            usr_cmd_->target_mode = uint8_t(RobotMotionState::RLControlMode);
                            std::cout << "[DDS][MODE] RL Control" << std::endl;
                        }
                        break;
                    case KeyCode::R1:
                        if (msfb_->GetCurrentState() == RobotMotionState::StandingUp
                            || msfb_->GetCurrentState() == RobotMotionState::RLControlMode) {
                            usr_cmd_->target_mode = uint8_t(RobotMotionState::LieDown);
                            std::cout << "[DDS][MODE] Lie Down" << std::endl;
                        }
                        break;
                    case KeyCode::R2:
                        usr_cmd_->target_mode = uint8_t(RobotMotionState::JointDamping);
                        std::cout << "[DDS][MODE] Joint Damping" << std::endl;
                        break;
                    default:
                        break;
                }
            }
        });

    std::cout << "\n╔════════════════════════════════════════════════╗\n"
              << "║              DDS COMMAND INTERFACE              ║\n"
              << "╚════════════════════════════════════════════════╝\n"
              << "  Topics:\n"
              << "    /STEER      → x=Forward, y=Side, yaw=Yaw\n"
              << "    /GAMEPAD_KEY → key name string\n"
              << "  Keys: L1=Stand  L2=RL Ctrl  R1=Lie Down  R2=Damping\n"
              << "\n";
}

void DdsCommandInterface::Stop() {
    if (ros_) {
        ros_->steer_sub.reset();
        ros_->key_sub.reset();
    }
    usr_cmd_->forward_vel_scale = 0.0f;
    usr_cmd_->side_vel_scale = 0.0f;
    usr_cmd_->turnning_vel_scale = 0.0f;
}

// 静态成员初始化（G20 + G12 兼容映射）
std::unordered_map<std::string, KeyCode> DdsCommandInterface::keyMap = {
    {"G20_KEY_L1", KeyCode::L1},
    {"G20_KEY_L2", KeyCode::L2},
    {"G20_KEY_R1", KeyCode::R1},
    {"G20_KEY_R2", KeyCode::R2},
    {"G12_KEY_C", KeyCode::L1},
    {"G12_KEY_A", KeyCode::L2},
    {"G12_KEY_B", KeyCode::R1},
    {"G12_KEY_D", KeyCode::R2}};
