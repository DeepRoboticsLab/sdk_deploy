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

        mHandleMap_[100][100] = std::bind(&GamepadInterface::Handle_Heartbeat, this, std::placeholders::_1,
                                          std::placeholders::_2, std::placeholders::_3);
        mHandleMap_[2][21] = std::bind(&GamepadInterface::Handle_Steer, this, std::placeholders::_1,
                                       std::placeholders::_2, std::placeholders::_3);
        mHandleMap_[1101][12] = std::bind(&GamepadInterface::Handle_Key, this, std::placeholders::_1,
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
    int ParseHeader(const char* buffer, int len) {
        const int headerSize = 16;
        if (len < headerSize) return -1;

        if ((uint8_t)buffer[0] == 0xeb && (uint8_t)buffer[2] == 0xeb && (uint8_t)buffer[3] == 0x90) {
            if ((uint8_t)buffer[1] != 0x90 && (uint8_t)buffer[1] != 0x91) {
                std::cout<<"Header format error"<<std::endl;
                return -1;
            }
            int bodyLen = ((uint16_t)buffer[5] << 8) | (uint8_t)buffer[4];
            return bodyLen + headerSize;
        }
        std::cout<<"Header magic error"<<std::endl;
        return -1;
    }

    bool ParseHeader(const char* buffer, int len, int& data_id, char* body, FromType& fromType) {
        const int headerSize = 16;
        if (len < headerSize) return false;

        if ((uint8_t)buffer[0] == 0xeb && (uint8_t)buffer[2] == 0xeb && (uint8_t)buffer[3] == 0x90) {
            if ((uint8_t)buffer[1] == 0x90) {
                fromType = FromType::inside;
            } else if ((uint8_t)buffer[1] == 0x91) {
                fromType = FromType::user;
            } else {
                return false;
            }

            int bodyLen = ((uint16_t)buffer[5] << 8) | (uint8_t)buffer[4];
            data_id = ((uint16_t)buffer[7] << 8) | (uint8_t)buffer[6];

            if (bodyLen != len - headerSize) {
                std::cout << "recv lost! bodyLen=" << bodyLen << " real=" << len - headerSize << std::endl;
                return false;
            }

            uint8_t dateFlag = buffer[8] & 0x03;
            if (dateFlag != 0x01) {
                std::cout << "Data format error" << std::endl;
                return false;
            }

            memcpy(body, buffer + headerSize, bodyLen);
            body[bodyLen] = '\0';
        } else {
            std::cout << "Header sync error" << std::endl;
            return false;
        }
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
            std::cout<<"JSON parse failed"<<std::endl;
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
            response = buildResponse(cmd_info, 1, "Unknown command");
        }
        return true;
    }

    void AddDataHeader(const std::string& in, std::string& out, int data_id = 0, NetInfo perNetInfo = {0}) {
        out.resize(16);

        out[0] = 0xeb;
        if (perNetInfo.fromType == FromType::inside) {
            out[1] = 0x90;
        } else {
            out[1] = 0x91;
        }
        out[2] = 0xeb;
        out[3] = 0x90;

        auto size = in.size();

        out[4] = (uint8_t)(size & 0xFF);
        out[5] = (uint8_t)((size >> 8) & 0xFF);

        out[6] = (uint8_t)(data_id & 0xFF);
        out[7] = (uint8_t)((data_id >> 8) & 0xFF);

        out[8] = 1;

        for (int i = 9; i < 16; i++) out[i] = 0x00;

        out += in;
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

        res = buildResponse(cmd_info, 0);
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
        static std::unordered_map<std::string, KeyCode> keyMap = {{"G20_KEY_L1", KeyCode::L1},
                                                                  {"G20_KEY_L2", KeyCode::L2},
                                                                  {"G20_KEY_R1", KeyCode::R1},
                                                                  {"G20_KEY_R2", KeyCode::R2}};

        if (items.contains("KeyList") && items["KeyList"].is_array()) {
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
