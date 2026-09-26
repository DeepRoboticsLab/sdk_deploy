
#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <iomanip>
#include <vector>
#include <deque>
#include <map>
#include <cmath>
#include <memory>
#include <thread>
#include <algorithm>
#include <sstream>
#include <fstream>
#include <sys/timerfd.h>
#include <sys/epoll.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdint.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <stdio.h>
#include <csignal>
#include <pthread.h>


namespace types {
using Vec3f = Eigen::Vector3f;
using Vec3d = Eigen::Vector3d;
using Vec4f = Eigen::Vector4f;
using Vec4d = Eigen::Vector4d;
using VecXf = Eigen::VectorXf;
using VecXd = Eigen::VectorXd;

using Mat3f = Eigen::Matrix3f;
using Mat3d = Eigen::Matrix3d;
using MatXf = Eigen::MatrixXf;
using MatXd = Eigen::MatrixXd;

const float gravity = 9.815;

struct RobotBasicState {
    Vec3f base_rpy;
    Vec4f base_quat;
    Mat3f base_rot_mat;
    Vec3f base_omega;
    Vec3f base_acc;
    MatXf flt_base_acc_mat;
    VecXf joint_pos;
    VecXf joint_vel;
    VecXf joint_tau;

    RobotBasicState(int dof = 16) {
        base_rpy.setZero();
        base_quat.setZero();
        base_rot_mat.setIdentity();
        base_omega.setZero();
        base_acc.setZero();
        flt_base_acc_mat = Eigen::MatrixXf::Zero(20, 3);
        joint_pos = VecXf::Zero(dof);
        joint_vel = VecXf::Zero(dof);
        joint_tau = VecXf::Zero(dof);
    }
};

struct RobotAction {
    VecXf goal_joint_pos;
    VecXf goal_joint_vel;
    VecXf kp;
    VecXf kd;
    VecXf tau_ff;

    MatXf ConvertToMat() {
        MatXf res(goal_joint_pos.rows(), 5);
        res.col(0) = kp;
        res.col(1) = goal_joint_pos;
        res.col(2) = kd;
        res.col(3) = goal_joint_vel;
        res.col(4) = tau_ff;
        return res;
    }
};


struct UserCommand {
    double time_stamp;
        uint8_t safe_control_mode; //0 normal, 1 stand, 2 sit, 3 joint damping
    uint8_t target_mode;
    float forward_vel_scale = 0;
    float side_vel_scale = 0;
    float turnning_vel_scale = 0;
    float reserved_scale;
};

enum FromType {
    inside = 0,   // 内部：手柄/巡检平台下发的指令
    user,         // 用户：用户下发的指令
};

enum class HandleType : uint8_t { xml = 0x00, json = 0x01 };

// 协议头常量定义（与实际手柄设备协议对齐）
constexpr int kHeaderSize = 16;
constexpr int kMaxPacketCount = 255;
constexpr uint8_t kMagicByte0 = 0xeb;
constexpr uint8_t kMagicByte2 = 0xeb;
constexpr uint8_t kMagicByte3 = 0x90;
constexpr uint8_t kFromTypeInside = 0x90;
constexpr uint8_t kFromTypeUser = 0x91;
constexpr uint8_t kProtocolVersion = 0x01;

// 协议头字节索引（与实际手柄设备对齐）
// 格式: [0xeb][FromType][0xeb][0x90][bodyLen_lo][bodyLen_hi][dataId_lo][dataId_hi][dataFlag][reserved x7]
enum HeaderIndex {
    IDX_MAGIC0 = 0,         // 0xeb
    IDX_FROM_TYPE_BYTE = 1, // 0x90(inside) / 0x91(user)
    IDX_MAGIC1 = 2,         // 0xeb
    IDX_MAGIC2 = 3,         // 0x90
    IDX_BODYLEN_LO = 4,
    IDX_BODYLEN_HI = 5,
    IDX_DATAID_LO = 6,
    IDX_DATAID_HI = 7,
    IDX_DATATYPE = 8,
    IDX_FRAME_PACKETID = 9,
    IDX_PROTOCOL_VERSION = 10,
    IDX_RESERVED_START = 11,
};

// Type 高位定义（参考 basic_server）
enum TypeHigh : uint32_t {
    BaseTypeHigh = 0x0000,
    CA9TypeHigh  = 0x0010,
    CC3TypeHigh  = 0x0020,
    CR1TypeHigh  = 0x0030,
};

// Type 低位定义
enum TypeLow : uint32_t {
    BasicTypeLow      = 0x0064,  // 100 - 基础模块
    MotionTypeLow     = 0x0001,  // 运动模块
    DeviceTypeLow     = 0x0002,  // 设备模块
    NavigationTypeLow = 0x0003,  // 导航模块
    AutoChargeTypeLow = 0x0004,  // 自主充电
    SDKTypeLow        = 0x0005,  // SDK/二开协议
};

// Command 高位定义
enum CommandHigh : uint32_t {
    PeriodCommandHigh = 0x00f0,  // 周期上报
};

// Command 低位定义
enum CommandLow : uint32_t {
    ReportCommandLow         = 0x0000,  // 主动上报
    GetCommandLow            = 0x0001,  // 查询
    SetCommandLow            = 0x0002,  // 修改
    AddCommandLow            = 0x0003,  // 增加
    DeleteCommandLow         = 0x0004,  // 删除
    PeriodDistributeCommandLow = 0x0005,  // 周期下发
};

// 常用组合 Type 值
constexpr uint32_t kBasicType   = (BaseTypeHigh << 16) | BasicTypeLow;      // 100
constexpr uint32_t kMotionType  = (BaseTypeHigh << 16) | MotionTypeLow;     // 1
constexpr uint32_t kDeviceType  = (BaseTypeHigh << 16) | DeviceTypeLow;     // 2
constexpr uint32_t kNavType     = (BaseTypeHigh << 16) | NavigationTypeLow; // 3
constexpr uint32_t kSDKType     = (BaseTypeHigh << 16) | SDKTypeLow;        // 5

// 常用组合 Command 值
constexpr uint32_t kCmdHeartbeat = (0x0000 << 16) | PeriodDistributeCommandLow;  // 5
constexpr uint32_t kCmdSteer     = (0x0010 << 16) | SetCommandLow;              // 1048578
constexpr uint32_t kCmdGet       = (0x0000 << 16) | GetCommandLow;              // 1
constexpr uint32_t kCmdSet       = (0x0000 << 16) | SetCommandLow;              // 2

// 帧信息（用于多包合并）
struct frameInfo_t {
    int frameLen{-1};       // 帧体长度
    int packetLen{-1};      // 当前包总长度
    uint16_t frameId{0};    // 帧ID
    uint8_t packetId{0};    // 分包序号
};

struct NetInfo {
    std::string ip;
    int port;
    FromType fromType;
};

struct DataInfo : public NetInfo {
    const char* buffer;
    ssize_t len;
};
struct CmdInfo : public NetInfo {
    int type;
    int command;
};
};  // namespace types
