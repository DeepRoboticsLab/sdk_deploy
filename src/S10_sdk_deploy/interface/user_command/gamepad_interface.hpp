/**
 * @file gamepad_interface.hpp
 * @brief gamepad
 * @author DeepRobotics
 * @version 1.0
 * @date 2026-01-07
 *
 * @copyright Copyright (c) 2025  DeepRobotics
 *
 */

#pragma once

#include <arpa/inet.h>
#include <unistd.h>

#include <atomic>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iomanip>
#include <iostream>
#include <list>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "common_types.h"
#include "custom_types.h"
#include "heartbeat.hpp"
#include "json.hpp"
#include "user_command_interface.h"

using json = nlohmann::json;

class GamepadInterface : public UserCommandInterface {
public:
    GamepadInterface(RobotName robot_name) : UserCommandInterface(robot_name) {
        mHeartbeat_ = std::make_unique<Heartbeat>();

        // 实际手柄设备使用 M系列 ProductID(0x0010) 作为 Type 高位
        constexpr uint32_t kCA9BasicType = (CA9TypeHigh << 16) | BasicTypeLow;   // 1048676
        constexpr uint32_t kCA9MotionType = (CA9TypeHigh << 16) | MotionTypeLow; // 1048577
        constexpr uint32_t kCA9SDKType   = (CA9TypeHigh << 16) | SDKTypeLow;     // 1048581

        // === 心跳 === type=1048676, command=5
        mHandleMap_[kCA9BasicType][kCmdHeartbeat] = std::bind(
            &GamepadInterface::Handle_Heartbeat, this, std::placeholders::_1,
            std::placeholders::_2, std::placeholders::_3);

        // === 摇杆(轴指令) === type=1048577(运动模块), command=1048578
        mHandleMap_[kCA9MotionType][kCmdSteer] = std::bind(
            &GamepadInterface::Handle_Steer, this, std::placeholders::_1,
            std::placeholders::_2, std::placeholders::_3);

        // === 按键 === type=1048581(SDK模块), command=1048578
        mHandleMap_[kCA9SDKType][kCmdSteer] = std::bind(
            &GamepadInterface::Handle_Key, this, std::placeholders::_1,
            std::placeholders::_2, std::placeholders::_3);
    }

    ~GamepadInterface() {}

    void Start() override {
        std::cout << "\n╔════════════════════════════════════════════════╗\n"
                  << "║                GAMEPAD TELEOP                  ║\n"
                  << "╚════════════════════════════════════════════════╝\n"
                  << "  Movement:  Left joystick\n"
                  << "  Rotation:  Right joystick\n"
                  << "  Mode:      R2 (damping)  L1 (stand)  L2 (control)\n"
                  << "\n";
    }

    void Stop() override {}
    UserCommand* GetUserCommand() override { return usr_cmd_; }
    /**
     * @brief 解析协议头，返回整帧长度（头+体）
     *
     * 协议头格式（16字节，与实际手柄设备对齐）:
     *   [0] 0xeb  [1] FromType(0x90/0x91)  [2] 0xeb  [3] 0x90
     *   [4-5] bodyLen(LE)  [6-7] dataId(LE)
     *   [8] dataFlag  [9-15] reserved
     */
    int ParseHeader(const char* buffer, int len) {
        if (len < kHeaderSize) return -1;

        // 检查魔数: [0]=0xeb, [2]=0xeb, [3]=0x90
        if ((uint8_t)buffer[IDX_MAGIC0] != kMagicByte0 ||
            (uint8_t)buffer[IDX_MAGIC1] != kMagicByte2 ||
            (uint8_t)buffer[IDX_MAGIC2] != kMagicByte3) {
            return -1;
        }

        // 检查 FromType 字节
        if ((uint8_t)buffer[IDX_FROM_TYPE_BYTE] != kFromTypeInside &&
            (uint8_t)buffer[IDX_FROM_TYPE_BYTE] != kFromTypeUser) {
            return -1;
        }

        int bodyLen = ((uint16_t)buffer[IDX_BODYLEN_HI] << 8) | (uint8_t)buffer[IDX_BODYLEN_LO];
        return bodyLen + kHeaderSize;
    }

    /**
     * @brief 解析协议头并提取帧信息（用于多包合并）
     */
    frameInfo_t ParseHeaderInfo(const char* buffer, int len) {
        frameInfo_t info;
        if (len < kHeaderSize) {
            info.frameLen = -1;
            return info;
        }

        if ((uint8_t)buffer[IDX_MAGIC0] != kMagicByte0 ||
            (uint8_t)buffer[IDX_MAGIC1] != kMagicByte2 ||
            (uint8_t)buffer[IDX_MAGIC2] != kMagicByte3) {
            info.frameLen = -1;
            return info;
        }

        if ((uint8_t)buffer[IDX_FROM_TYPE_BYTE] != kFromTypeInside &&
            (uint8_t)buffer[IDX_FROM_TYPE_BYTE] != kFromTypeUser) {
            info.frameLen = -1;
            return info;
        }

        int bodyLen = ((uint16_t)buffer[IDX_BODYLEN_HI] << 8) | (uint8_t)buffer[IDX_BODYLEN_LO];
        int data_id = ((uint16_t)buffer[IDX_DATAID_HI] << 8) | (uint8_t)buffer[IDX_DATAID_LO];

        info.frameLen = bodyLen;
        info.frameId = static_cast<uint16_t>(data_id);
        info.packetLen = len;
        info.packetId = static_cast<uint8_t>(buffer[IDX_FRAME_PACKETID]);
        return info;
    }

    bool ParseHeader(const char* buffer, int len, int& data_id, char* body, FromType& fromType) {
        if (len < kHeaderSize) return false;

        // 检查魔数: [0]=0xeb, [2]=0xeb, [3]=0x90
        if ((uint8_t)buffer[IDX_MAGIC0] != kMagicByte0 ||
            (uint8_t)buffer[IDX_MAGIC1] != kMagicByte2 ||
            (uint8_t)buffer[IDX_MAGIC2] != kMagicByte3) {
            return false;
        }

        // 解析 FromType
        if ((uint8_t)buffer[IDX_FROM_TYPE_BYTE] == kFromTypeInside) {
            fromType = FromType::inside;
        } else if ((uint8_t)buffer[IDX_FROM_TYPE_BYTE] == kFromTypeUser) {
            fromType = FromType::user;
        } else {
            return false;
        }

        int bodyLen = ((uint16_t)buffer[IDX_BODYLEN_HI] << 8) | (uint8_t)buffer[IDX_BODYLEN_LO];
        data_id = ((uint16_t)buffer[IDX_DATAID_HI] << 8) | (uint8_t)buffer[IDX_DATAID_LO];

        if (bodyLen != len - kHeaderSize) {
            return false;
        }

        // 拷贝 body 数据
        memcpy(body, buffer + kHeaderSize, bodyLen);
        body[bodyLen] = '\0';

        return true;
    }

    bool ParseData(const DataInfo& dataInfo, std::string& response) {
        int type = 0, command = 0;
        bool ret = false;

        mHeartbeat_->isConntected_ = true;
        mHeartbeat_->lastHeartbeatTime_ = std::chrono::steady_clock::now();

        json document;
        json items;

        ret = ParseJson(dataInfo.buffer, dataInfo.len, document, type, command, items);
        if (!ret) {
            response = "JSON format parsing failed";
            return false;
        }

        CmdInfo cmd_info;
        cmd_info.ip = dataInfo.ip;
        cmd_info.port = dataInfo.port;
        cmd_info.fromType = dataInfo.fromType;
        cmd_info.type = type;
        cmd_info.command = command;

        if (mHandleMap_[type][command]) {
            mHandleMap_[type][command](items, response, cmd_info);
        } else {
            // std::cout << "[CMD] Unknown command from " << cmd_info.ip
            //           << " Type=" << type << " Command=" << command << std::endl;
            response = buildResponse(cmd_info, 1, "Unknown command");
        }
        return true;
    }

    /**
     * @brief 打包协议头（与实际手柄设备协议对齐）
     *
     * @param in         响应 body 数据
     * @param out        输出：头 + body
     * @param data_id    数据ID
     * @param perNetInfo 对端网络信息
     * @param totalSize  分包场景下原始总大小（0 表示不分包）
     * @param packetId   分包序号
     */
    void PackingHeaderWithData(const std::string& in, std::string& out, int data_id = 0,
                                NetInfo perNetInfo = {0}, size_t totalSize = 0, uint8_t packetId = 0) {
        out.resize(kHeaderSize);

        // 魔数: [0]=0xeb, [2]=0xeb, [3]=0x90
        out[IDX_MAGIC0] = kMagicByte0;
        out[IDX_MAGIC1] = kMagicByte2;  // 0xeb
        out[IDX_MAGIC2] = kMagicByte3;  // 0x90

        // FromType: [1]=0x90/0x91
        if (perNetInfo.fromType == FromType::inside) {
            out[IDX_FROM_TYPE_BYTE] = kFromTypeInside;
        } else {
            out[IDX_FROM_TYPE_BYTE] = kFromTypeUser;
        }

        // body 长度（分包场景使用 totalSize）
        size_t size = (totalSize > 0) ? totalSize : in.size();
        out[IDX_BODYLEN_LO] = (uint8_t)(size & 0xFF);
        out[IDX_BODYLEN_HI] = (uint8_t)((size >> 8) & 0xFF);

        // data_id
        out[IDX_DATAID_LO] = (uint8_t)(data_id & 0xFF);
        out[IDX_DATAID_HI] = (uint8_t)((data_id >> 8) & 0xFF);

        // 数据类型：JSON
        out[IDX_DATATYPE] = 0x01;

        // 保留字节清零
        for (int i = IDX_FRAME_PACKETID; i < kHeaderSize; i++) {
            out[i] = 0x00;
        }

        out += in;
    }

    /**
     * @brief 将大响应拆分为多个分包（每个包体最大 max_packet_size）
     */
    void SplitRes(const std::string& in, std::vector<std::string>& out, size_t max_packet_size = 32 * 1024) {
        out.clear();
        if (in.size() <= max_packet_size) {
            out.push_back(in);
            return;
        }
        size_t offset = 0;
        while (offset < in.size()) {
            size_t chunkLen = std::min(max_packet_size, in.size() - offset);
            out.push_back(in.substr(offset, chunkLen));
            offset += chunkLen;
        }
    }

private:
    std::unique_ptr<Heartbeat> mHeartbeat_;

    using HANDLE_FUNC = std::function<void(json& item, std::string& res, const CmdInfo& cmd_info)>;

    std::unordered_map<int, std::unordered_map<int, HANDLE_FUNC>> mHandleMap_;

    bool ParseJson(const char* buf, int len, json& document, int& type, int& command, json& items) {
        if (!buf || len == 0) return false;

        try {
            document = json::parse(buf, buf + len);

            if (!document.is_object() || !document.contains("PatrolDevice"))
                throw std::runtime_error("No PatrolDevice");

            json& root = document["PatrolDevice"];
            if (!root.is_object()) throw std::runtime_error("PatrolDevice not object");

            if (!root.contains("Items")) throw std::runtime_error("No Items");

            items = root["Items"];

            if (!root.contains("Type") || !root.contains("Command")) throw std::runtime_error("No Type/Command");

            if (!root["Type"].is_number_integer() || !root["Command"].is_number_integer())
                throw std::runtime_error("Type/Command not Int");

            type = root["Type"].get<int>();
            command = root["Command"].get<int>();

        } catch (const json::parse_error& e) {
            std::cerr << "JSON Parse Error: " << e.what() << std::endl;
            return false;
        } catch (const std::exception& e) {
            std::cerr << "JSON Logic Error: " << e.what() << std::endl;
            return false;
        }
        return true;
    }

    void JSONRootToBroker(const int& type, const int& command, json& root, bool isNew = false) {
        if (isNew) {
            root = json::object();
            root["PatrolDevice"] = {
                {"Items", json::object()}, {"Type", type}, {"Command", command}, {"Time", getCurrentDateTime()}};
        } else {
            root["PatrolDevice"]["Type"] = type;
            root["PatrolDevice"]["Command"] = command;
            root["PatrolDevice"]["Time"] = getCurrentDateTime();
        }
    }

    std::string buildResponse(const CmdInfo& cmd_info, const int& errorCode, std::string errorMsg = "") {
        std::string res;
        json document;

        JSONRootToBroker(cmd_info.type, cmd_info.command, document, true);

        document["PatrolDevice"]["Items"]["ErrorCode"] = errorCode;

        if (!errorMsg.empty()) {
            document["PatrolDevice"]["Items"]["ErrorMessage"] = errorMsg;
        }

        res = document.dump();
        return res;
    }

    void Handle_Heartbeat(json& items, std::string& res, const CmdInfo& cmd_info) {
        NetInfo netInfo;
        netInfo.ip = cmd_info.ip;
        netInfo.port = cmd_info.port;
        netInfo.fromType = cmd_info.fromType;
        mHeartbeat_->wakeUp(netInfo);

        // std::cout << "[CMD] Heartbeat from " << cmd_info.ip << ":" << cmd_info.port << std::endl;
        res = buildResponse(cmd_info, 0);
    }

    void Handle_SteerOrKey(json& items, std::string& res, const CmdInfo& cmd_info) {
        if (items.contains("KeyList")) {
            Handle_Key(items, res, cmd_info);
        } else {
            Handle_Steer(items, res, cmd_info);
        }
    }

    void Handle_Steer(json& items, std::string& res, const CmdInfo& cmd_info) {
        if (!items.contains("X") || !items.contains("Y")) {
            res = buildResponse(cmd_info, 2, "Missing fields");
            return;
        }

        try {
            if (items["X"].is_number()) usr_cmd_->forward_vel_scale = items["X"].get<float>();
            if (items["Y"].is_number()) usr_cmd_->side_vel_scale = items["Y"].get<float>();
            if (items.contains("Yaw") && items["Yaw"].is_number())
                usr_cmd_->turnning_vel_scale = items["Yaw"].get<float>();
        } catch (const std::exception& e) {
            res = buildResponse(cmd_info, 3, "Data type error");
            return;
        }

        std::cout << "[CMD] Steer from " << cmd_info.ip
                  << " X=" << usr_cmd_->forward_vel_scale
                  << " Y=" << usr_cmd_->side_vel_scale
                  << " Yaw=" << usr_cmd_->turnning_vel_scale << std::endl;

        static std::string LastPerIp = "";
        static int64_t LastPerTime = 0;
        auto now = std::chrono::steady_clock::now();
        auto timestamp = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();

        if (LastPerIp == "" || LastPerIp == cmd_info.ip) {
            LastPerIp = cmd_info.ip;
            LastPerTime = timestamp;
        } else if (timestamp - LastPerTime > 2) {
            LastPerIp = cmd_info.ip;
            LastPerTime = timestamp;
        } else {
            std::cout << "IP Conflict" << std::endl;
            res = buildResponse(cmd_info, 4, "IP Conflict");
            return;
        }
    }

    void Handle_Key(json& items, std::string& res, const CmdInfo& cmd_info) {
        static std::unordered_map<std::string, KeyCode> keyMap = {{"G12_KEY_C", KeyCode::L1},
                                                                  {"G12_KEY_A", KeyCode::L2},
                                                                  {"G12_KEY_B", KeyCode::R1},
                                                                  {"G12_KEY_D", KeyCode::R2}};

        if (items.contains("KeyList") && items["KeyList"].is_array()) {
            std::cout << "[CMD] Key from " << cmd_info.ip << " keys=[";
            for (size_t i = 0; i < items["KeyList"].size(); i++) {
                if (i > 0) std::cout << ",";
                std::cout << items["KeyList"][i].get<std::string>();
            }
            std::cout << "]" << std::endl;

            for (const auto& key : items["KeyList"]) {
                if (key.is_string()) {
                    std::string keyStr = key.get<std::string>();

                    if (keyMap.count(keyStr)) {
                        KeyCode code = keyMap[keyStr];
                        switch (code) {
                            case KeyCode::L1:
                                if (msfb_->GetCurrentState() == RobotMotionState::WaitingForStand 
                                    || msfb_->GetCurrentState() == RobotMotionState::LieDown) {
                                    usr_cmd_->target_mode = uint8_t(RobotMotionState::StandingUp);
                                    std::cout << "[MODE] Standing Up\n";
                                }
                                break;
                            case KeyCode::L2:
                                if (msfb_->GetCurrentState() == RobotMotionState::StandingUp) {
                                    usr_cmd_->target_mode = uint8_t(RobotMotionState::RLControlMode);
                                    std::cout << "[MODE] RL Control\n";
                                }
                                break;
                            case KeyCode::R1:
                                if (msfb_->GetCurrentState() == RobotMotionState::StandingUp
                                    || msfb_->GetCurrentState() == RobotMotionState::RLControlMode) {
                                    usr_cmd_->target_mode = uint8_t(RobotMotionState::LieDown);
                                    std::cout << "[MODE] Lie Down\n";
                                }
                                break;
                            case KeyCode::R2:
                                usr_cmd_->target_mode = uint8_t(RobotMotionState::JointDamping);
                                std::cout << "[MODE] Joint Damping\n";
                                break;
                            default:
                                break;
                        }
                    }
                }
            }
        }
        res = buildResponse(cmd_info, 0);
    }

    std::string getCurrentDateTime() {
        auto now = std::chrono::system_clock::now();
        std::time_t currentTime = std::chrono::system_clock::to_time_t(now);
        std::tm* localTime = std::localtime(&currentTime);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
        std::stringstream ss;
        ss << std::put_time(localTime, "%Y-%m-%d %H:%M:%S") << "." << std::setw(3) << std::setfill('0') << ms.count();
        return ss.str();
    }
};
