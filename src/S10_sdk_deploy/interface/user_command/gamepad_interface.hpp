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
#include "user_command_interface.h"

class GamepadInterface : public UserCommandInterface {
public:
    GamepadInterface(RobotName robot_name);

    ~GamepadInterface();

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

    bool ParseData(const DataInfo& dataInfo, std::string& response);

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
    struct JsonHandlers;
    std::unique_ptr<JsonHandlers> handlers_;
};
