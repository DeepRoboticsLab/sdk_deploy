/**
 * @file multi_pkt_frame.hpp
 * @brief 多包帧合并处理（与 basic_server MultiPktFrame 对齐）
 * @author DeepRobotics
 * @version 1.0
 * @date 2026-01-07
 *
 * @copyright Copyright (c) 2025  DeepRobotics
 *
 */
#pragma once

#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

#include "common_types.h"

class MultiPktFrame {
public:
    struct FrameBuffer {
        int frame_Id{0};
        size_t total_length{0};
        size_t received_length{0};
        std::map<int, std::vector<char>> packets;
        bool has_header{false};
        uint64_t lastTime{0};
    };

    MultiPktFrame() {
        mClearBufferThread_ = std::make_unique<std::thread>(&MultiPktFrame::clearBuffer, this);
    }

    ~MultiPktFrame() {
        // 标记退出（简单处理，线程会在超时逻辑中自然退出）
        mRecvBufferMap_.clear();
    }

    /**
     * @brief 处理一个分包，返回 true 表示该帧已完整
     */
    bool dealMultiPacket(frameInfo_t& frameInfo, std::shared_ptr<char> data) {
        int recvFrameId = frameInfo.frameId;
        int recvPacketId = frameInfo.packetId;
        size_t packet_data_len = frameInfo.packetLen - kHeaderSize;

        auto now = std::chrono::system_clock::now();
        auto timestamp = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();

        std::lock_guard<std::mutex> lock(mBufferMutex_);
        auto& frame_buffer = mRecvBufferMap_[recvFrameId];
        frame_buffer.frame_Id = recvFrameId;
        frame_buffer.total_length = frameInfo.frameLen + kHeaderSize;
        frame_buffer.lastTime = timestamp;

        // 包编号有效性检查
        if (recvPacketId < 0 || recvPacketId > kMaxPacketCount) {
            std::cout << "[MultiPkt] Invalid packet ID: " << recvPacketId << std::endl;
            return false;
        }

        // 去重
        if (frame_buffer.packets.find(recvPacketId) != frame_buffer.packets.end()) {
            std::cout << "[MultiPkt] Duplicate packet ID: " << recvPacketId << std::endl;
            return false;
        }

        // 存储数据包：packetId==0 包含完整协议头，后续包只含 body
        if (recvPacketId == 0) {
            frame_buffer.has_header = true;
            std::vector<char> packet_data(data.get(), data.get() + frameInfo.packetLen);
            frame_buffer.packets[recvPacketId] = packet_data;
            frame_buffer.received_length += packet_data.size();
        } else {
            std::vector<char> packet_data(data.get() + kHeaderSize, data.get() + frameInfo.packetLen);
            frame_buffer.packets[recvPacketId] = packet_data;
            frame_buffer.received_length += packet_data.size();
        }

        return checkFrameComplete(recvFrameId);
    }

    /**
     * @brief 将已完整的帧拼接为连续 buffer
     */
    std::shared_ptr<char> assembleAndProcessFrame(int frame_id) {
        std::lock_guard<std::mutex> lock(mBufferMutex_);
        auto it = mRecvBufferMap_.find(frame_id);
        if (it == mRecvBufferMap_.end()) {
            return nullptr;
        }
        auto& frame_buffer = it->second;
        std::shared_ptr<char> complete_buffer =
            std::shared_ptr<char>(new char[frame_buffer.total_length], [](char* p) { delete[] p; });

        size_t current_pos = 0;
        for (const auto& packet_pair : frame_buffer.packets) {
            const auto& packet_data = packet_pair.second;
            size_t packet_size = packet_data.size();

            if (current_pos + packet_size > frame_buffer.total_length) {
                std::cout << "[MultiPkt] Assembly overflow!" << std::endl;
                return nullptr;
            }

            memcpy(complete_buffer.get() + current_pos, packet_data.data(), packet_size);
            current_pos += packet_size;
        }

        if (current_pos != frame_buffer.total_length) {
            std::cout << "[MultiPkt] Length mismatch: expected=" << frame_buffer.total_length
                      << " actual=" << current_pos << std::endl;
            return nullptr;
        }

        return complete_buffer;
    }

    void clearFrameBuffer(int frame_id) {
        std::lock_guard<std::mutex> lock(mBufferMutex_);
        mRecvBufferMap_.erase(frame_id);
    }

private:
    bool checkFrameComplete(int frame_id) {
        auto it = mRecvBufferMap_.find(frame_id);
        if (it == mRecvBufferMap_.end()) return false;
        auto& frame_buffer = it->second;

        if (frame_buffer.received_length < frame_buffer.total_length) return false;

        // 检查包编号连续性
        int expected_packet = 0;
        for (const auto& packet_pair : frame_buffer.packets) {
            if (packet_pair.first != expected_packet) {
                std::cout << "[MultiPkt] Packet ID not contiguous, expected=" << expected_packet
                          << " actual=" << packet_pair.first << std::endl;
                return false;
            }
            expected_packet++;
        }

        if (!frame_buffer.has_header) {
            std::cout << "[MultiPkt] Missing header packet (ID=0)" << std::endl;
            return false;
        }

        return true;
    }

    /**
     * @brief 后台线程：清理超时帧缓冲区（>3秒未更新）
     */
    void clearBuffer() {
        while (true) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            auto now = std::chrono::system_clock::now();
            auto timestamp = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();

            std::lock_guard<std::mutex> lock(mBufferMutex_);
            for (auto it = mRecvBufferMap_.begin(); it != mRecvBufferMap_.end();) {
                if (timestamp - it->second.lastTime > 3) {
                    std::cout << "[MultiPkt] Frame " << it->first << " timeout, auto-cleared" << std::endl;
                    it = mRecvBufferMap_.erase(it);
                } else {
                    ++it;
                }
            }
        }
    }

    std::unique_ptr<std::thread> mClearBufferThread_;
    std::unordered_map<int, FrameBuffer> mRecvBufferMap_;
    std::mutex mBufferMutex_;
};
