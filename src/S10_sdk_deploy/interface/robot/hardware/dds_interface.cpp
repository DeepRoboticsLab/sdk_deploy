#include "common_types.h"
using namespace types;
#include "robot_interface.h"
using namespace interface;
#include "dds_interface.hpp"

#include <rclcpp/node.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/time.hpp>

#include "drdds/msg/battery_data.hpp"
#include "drdds/msg/imu_data.hpp"
#include "drdds/msg/joints_data.hpp"
#include "drdds/msg/joints_data_cmd.hpp"

struct DdsInterface::RosImpl {
    rclcpp::Publisher<drdds::msg::JointsDataCmd>::SharedPtr joint_cmd_pub;
    rclcpp::GenericSubscription::SharedPtr joint_data_sub;
    rclcpp::GenericSubscription::SharedPtr imu_data_sub;
    rclcpp::GenericSubscription::SharedPtr health_data_sub;

    rclcpp::Serialization<drdds::msg::JointsData> joints_ser;
    rclcpp::Serialization<drdds::msg::ImuData> imu_ser;
    rclcpp::Serialization<drdds::msg::BatteryData> bat_ser;
};

DdsInterface::DdsInterface(const std::string& robot_name, const int& dof_num)
    : RobotInterface(robot_name, dof_num), ros_(std::make_unique<RosImpl>()) {
    std::cout << robot_name << " is using Ecan Hardware Interface" << std::endl;
    joint_pos_ = VecXf::Zero(dof_num_);
    joint_vel_ = VecXf::Zero(dof_num_);
    joint_tau_ = VecXf::Zero(dof_num_);
    motor_temperture_ = VecXf::Zero(dof_num_);
    driver_temperture_ = VecXf::Zero(dof_num_);

    pos_offset_ = new float[dof_num_];
    joint_dir_ = new float[dof_num_];
    data_updated_ = new bool[dof_num_];
    joint_config_ = new JointConfig[dof_num_];
    control_word = new uint16_t[4];

    driver_status_.resize(dof_num_);
    joint_data_id_.resize(dof_num_);
    battery_data_.resize(BATTERY_DATA_SIZE);

    ros_->joint_cmd_pub = node_->create_publisher<drdds::msg::JointsDataCmd>("/JOINTS_CMD", 10);

    ros_->joint_data_sub = node_->create_generic_subscription(
        "/JOINTS_DATA", "drdds/msg/JointsData", 10,
        [this](std::shared_ptr<rclcpp::SerializedMessage> raw) {
            drdds::msg::JointsData msg;
            ros_->joints_ser.deserialize_message(raw.get(), &msg);
            ++run_cnt_;
            for (int i = 0; i < dof_num_; ++i) {
                joint_pos_(i) = msg.data.joints_data[i].position * joint_dir_[i] + pos_offset_[i] / 180. * M_PI;
                joint_vel_(i) = msg.data.joints_data[i].velocity * joint_dir_[i];
                joint_tau_(i) = msg.data.joints_data[i].torque * joint_dir_[i];
                motor_temperture_(i) = float(msg.data.joints_data[i].motion_temp);
                driver_temperture_(i) = float(msg.data.joints_data[i].driver_temp);
                driver_status_[i] = msg.data.joints_data[i].status_word;
                joint_data_id_[i] = uint16_t(run_cnt_);
            }
            ri_ts_ = rclcpp::Time(msg.header.stamp).seconds();
        });

    ros_->imu_data_sub = node_->create_generic_subscription(
        "/IMU_DATA", "drdds/msg/ImuData", 10,
        [this](std::shared_ptr<rclcpp::SerializedMessage> raw) {
            drdds::msg::ImuData msg;
            ros_->imu_ser.deserialize_message(raw.get(), &msg);
            constexpr float kDeg2Rad = static_cast<float>(M_PI / 180.0);
            rpy_ = Vec3f(msg.data.roll * kDeg2Rad, msg.data.pitch * kDeg2Rad, msg.data.yaw * kDeg2Rad);
            acc_ << msg.data.acc_x, msg.data.acc_y, msg.data.acc_z;
            omega_body_ << msg.data.omega_x, msg.data.omega_y, msg.data.omega_z;
            imu_ts_ = rclcpp::Time(msg.header.stamp).seconds();
        });

    ros_->health_data_sub = node_->create_generic_subscription(
        "/BATTERY_DATA", "drdds/msg/BatteryData", 10,
        [this](std::shared_ptr<rclcpp::SerializedMessage> raw) {
            drdds::msg::BatteryData msg;
            ros_->bat_ser.deserialize_message(raw.get(), &msg);
            battery_data_[0] = uint8_t(msg.data[0].battery_level);
            remain_bat = battery_data_[0] / 100;
            battery_info_[0].battery_level = msg.data[0].battery_level;
        });

    sleep(1);

    control_word[0] = kIndexDisable;
    control_word[1] = kIndexErrorReset;
    control_word[2] = kIndexEnable;
    control_word[3] = kIndexGetStatusWord;
    ResetJointError();
}

DdsInterface::~DdsInterface() {
    delete[] pos_offset_;
    delete[] joint_dir_;
    delete[] data_updated_;
    delete[] joint_config_;
}

void DdsInterface::ResetPositionOffset() {
    memset(data_updated_, 0, dof_num_ * sizeof(bool));
    this->SetJointCommand(MatXf::Zero(dof_num_, 5));
    usleep(100 * 1000);
    VecXf last_joint_pos = this->GetJointPosition();
    VecXf current_joint_pos = this->GetJointPosition();
    int cnt = 0;
    while (!IsDataUpdatedFinished()) {
        ++cnt;
        this->RefreshRobotData();
        this->SpinSome();
        current_joint_pos = this->GetJointPosition();

        for (int i = 0; i < dof_num_; ++i) {
            if (!data_updated_[i] && current_joint_pos(i) != last_joint_pos(i)) {
                data_updated_[i] = true;
            }
        }
        last_joint_pos = current_joint_pos;
        usleep(1000);
        if (cnt == 10000) {
            std::cout << data_updated_ << std::endl;
            std::cout << "joint data update is not finished\n";
        }
    }
    for (int i = 1; i < dof_num_; i += 4) {
        if (current_joint_pos(i) < -210. / 180. * M_PI) {
            pos_offset_[i] = pos_offset_[i] + 360.;
            std::cout << "joint " << i << " offset is changed to " << pos_offset_[i] << "\n";
        } else if (current_joint_pos(i) > 90. / 180. * M_PI) {
            pos_offset_[i] = pos_offset_[i] - 360.;
            std::cout << "joint " << i << " offset is changed to " << pos_offset_[i] << "\n";
        }
    }
    for (int i = 0; i < dof_num_; ++i) {
        joint_config_[i].dir = joint_dir_[i];
        joint_config_[i].offset = pos_offset_[i] * static_cast<float>(M_PI / 180.0);
    }
}

void DdsInterface::ResetJointError() {
    auto msg = drdds::msg::JointsDataCmd();

    for (int j = 0; j < 4; ++j) {
        for (int i = 0; i < dof_num_; ++i) {
            msg.data.joints_data[i].position = 0;
            msg.data.joints_data[i].velocity = 0;
            msg.data.joints_data[i].torque = 0;
            msg.data.joints_data[i].kp = 0;
            msg.data.joints_data[i].kd = 0;
            msg.data.joints_data[i].control_word = control_word[j];
        }
        std::cout << "ResetJointError publish!! \n" << std::endl;
        ros_->joint_cmd_pub->publish(msg);
        usleep(10 * 1000);
    }
}

void DdsInterface::PublishJointCommand(const Eigen::Matrix<float, Eigen::Dynamic, 5>& input, bool zero_kp_for_wheel) {
    auto msg = drdds::msg::JointsDataCmd();
    for (int i = 0; i < dof_num_; ++i) {
        msg.data.joints_data[i].position =
                (input(i, 1) - joint_config_[i].offset) * joint_config_[i].dir;
        msg.data.joints_data[i].velocity = input(i, 3) * joint_dir_[i];
        msg.data.joints_data[i].torque = input(i, 4) * joint_dir_[i];
        msg.data.joints_data[i].kp = (zero_kp_for_wheel && i % 4 == 3) ? 0.0f : input(i, 0);
        msg.data.joints_data[i].kd = input(i, 2);
        msg.data.joints_data[i].control_word = kIndexMotorControl;
    }
    if (!zero_kp_for_wheel) {
        std::cout << "SetJointCommand publish!! \n" << std::endl;
    }
    ros_->joint_cmd_pub->publish(msg);
}

void DdsInterface::SetJointCommand(Eigen::Matrix<float, Eigen::Dynamic, 5> input) {
    joint_cmd_ = input;
    PublishJointCommand(input, false);
}
