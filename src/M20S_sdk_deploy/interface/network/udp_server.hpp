/**
 * @file udp_server.hpp
 * @brief udp server
 * @author DeepRobotics
 * @version 1.0
 * @date 2026-01-07
 *
 * @copyright Copyright (c) 2025  DeepRobotics
 *
 */

#pragma once

#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>

#include <atomic>
#include <cstring>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "common_types.h"
#include "gamepad_interface.hpp"
#include "thread_pool.hpp"

#define UDP_PORT 30000
#define IP "0.0.0.0"
#define UDP_BUFFER_SIZE 1024 * 32

class ThreadPool;
class GamepadInterface;

class UdpServer {
public:
    UdpServer(GamepadInterface* gp) : mGamepadInterface_(gp) {
        mThreadPool_ = std::make_unique<ThreadPool>(10);

        if (!Init()) {
            std::cerr << "[UdpServer] Init Failed!" << std::endl;
        }

        mIsRunning_ = true;
        mServerThread_ = std::thread(&UdpServer::RecvLoop, this);
        std::cout << "UDP Server started, listening on port: " << UDP_PORT << std::endl;
    }

    ~UdpServer() {
        mIsRunning_ = false;

        if (mServerThread_.joinable()) {
            mServerThread_.join();
        }

        if (mFd >= 0) {
            std::cout << "Closing UDP socket..." << std::endl;
            close(mFd);
            mFd = -1;
        }
    }

private:
    bool Init() {
        mFd = socket(AF_INET, SOCK_DGRAM, 0);
        if (mFd < 0) {
            perror("Create socket failed");
            return false;
        }

        int bufSize = 1024 * 1024 * 25;
        setsockopt(mFd, SOL_SOCKET, SO_RCVBUF, &bufSize, sizeof(bufSize));
        setsockopt(mFd, SOL_SOCKET, SO_SNDBUF, &bufSize, sizeof(bufSize));

        struct timeval timeout;
        timeout.tv_sec = 5;
        timeout.tv_usec = 0;
        if (setsockopt(mFd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
            perror("Set timeout failed");
            close(mFd);
            return false;
        }

        int opt = 1;
        setsockopt(mFd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        sockaddr_in serverAddr{};
        serverAddr.sin_family = AF_INET;
        serverAddr.sin_addr.s_addr = inet_addr(IP);
        serverAddr.sin_port = htons(UDP_PORT);

        if (bind(mFd, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
            perror("Bind failed");
            close(mFd);
            return false;
        }

        return true;
    }

    void RecvLoop() {
        struct sockaddr_in client_addr;
        socklen_t client_addr_len = sizeof(client_addr);

        std::vector<char> buffer(UDP_BUFFER_SIZE);

        while (mIsRunning_) {
            ssize_t recvLen =
                recvfrom(mFd, buffer.data(), UDP_BUFFER_SIZE, 0, (struct sockaddr*)&client_addr, &client_addr_len);

            if (recvLen > 0) {
                auto data = std::make_shared<std::vector<char>>(buffer.begin(), buffer.begin() + recvLen);

                auto headerLen = mGamepadInterface_->ParseHeader(data->data(), recvLen);
                if (headerLen > 0 && headerLen == recvLen) {
                    dataHandle(mFd, data, headerLen, client_addr);
                } else {
                    std::cout<<"Invalid header"<<std::endl;
                }
            }
        }
    }

    void dataHandle(int socket, std::shared_ptr<std::vector<char>> data, int dataLen, const sockaddr_in& clientAddr) {
        mThreadPool_->enqueue([=]() {
            FromType from_type;
            bool ret = false;
            int dataId = -1;
            std::string res;
            DataInfo dataInfo;

            if (mGamepadInterface_->ParseHeader(data->data(), dataLen, dataId, data->data(), from_type)) {
                std::string ip = inet_ntoa(clientAddr.sin_addr);
                int port = ntohs(clientAddr.sin_port);

                dataInfo.ip = ip;
                dataInfo.port = port;
                dataInfo.fromType = from_type;
                dataInfo.buffer = data->data();
                dataInfo.len = dataLen;

                ret = mGamepadInterface_->ParseData(dataInfo, res);
            } else {
                res = "Protocol header error,error code:0x2";
            }

            if (res.length() > 0) {
                std::string Response;
                mGamepadInterface_->AddDataHeader(res, Response, dataId, dataInfo);
                sendData(socket, Response.c_str(), Response.length(), clientAddr);
            }
        });
    }

    void sendData(int socket, const char* data, int dataLen, const sockaddr_in& clientAddr) {
        if (dataLen > 0) {
            ssize_t sentBytes = 0;
            while (sentBytes < dataLen) {
                ssize_t sendNum = sendto(socket, data + sentBytes, dataLen - sentBytes, 0,
                                         (struct sockaddr*)&clientAddr, sizeof(clientAddr));
                if (sendNum < 0) {
                    std::cout << "Send data failure: " << strerror(errno) << std::endl;
                    return;
                }
                sentBytes += sendNum;
            }
        }
    }

private:
    int mFd = -1;
    std::atomic<bool> mIsRunning_{false};
    std::thread mServerThread_;

    std::unique_ptr<ThreadPool> mThreadPool_;
    std::unique_ptr<GamepadInterface> mGamepadInterface_;
};