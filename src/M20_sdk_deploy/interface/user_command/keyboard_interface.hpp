// keyboard_interface.hpp
#pragma once

#include "user_command_interface.h"
#include "custom_types.h"
#include <thread>
#include <atomic>
#include <unordered_map>
#include <unordered_set>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>
#include <chrono>
#include <cctype>
#include <mutex>
#include <vector>

using namespace interface;
using namespace types;

class KeyboardInterface : public UserCommandInterface
{
private:
    std::atomic<bool> running_{false};
    std::thread kb_thread_;
    mutable std::mutex keys_mutex_;

    float max_forward_ = 0.7f;
    float max_side_    = 0.5f;
    float max_yaw_     = 0.7f;

    std::unordered_set<char> held_keys_;
    std::unordered_map<char, double> last_seen_time_;
    
    const std::unordered_set<char> velocity_keys_ = {'w', 's', 'a', 'd', 'q', 'e'};
    const double key_timeout_ms_ = 500.0;

    void ClipNumber(float& num, float low, float high)
    {
        if (num < low) num = low;
        if (num > high) num = high;
    }

    double GetCurrentTimeStamp()
    {
        static auto start = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        return std::chrono::duration<double, std::milli>(now - start).count();
    }

    static void setup_raw_mode()
    {
        termios t{};
        tcgetattr(STDIN_FILENO, &t);
        termios raw = t;
        raw.c_lflag &= ~(ECHO | ICANON);
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);
        
        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    }

    static void restore_terminal()
    {
        termios t{};
        tcgetattr(STDIN_FILENO, &t);
        t.c_lflag |= (ECHO | ICANON);
        tcsetattr(STDIN_FILENO, TCSANOW, &t);
    }

    void compute_velocity_from_held_keys(float& fwd, float& side, float& yaw)
    {
        fwd = 0.0f;
        side = 0.0f;
        yaw = 0.0f;

        std::lock_guard<std::mutex> lock(keys_mutex_);
        
        if (held_keys_.count('w')) fwd += max_forward_;
        if (held_keys_.count('s')) fwd -= max_forward_;
        if (held_keys_.count('a')) side += max_side_;
        if (held_keys_.count('d')) side -= max_side_;
        if (held_keys_.count('q')) yaw += max_yaw_;
        if (held_keys_.count('e')) yaw -= max_yaw_;
        
        ClipNumber(fwd, -max_forward_, max_forward_);
        ClipNumber(side, -max_side_, max_side_);
        ClipNumber(yaw, -max_yaw_, max_yaw_);
    }

    void process_mode_command(char k)
    {
        if (k == 'r') {
            usr_cmd_->target_mode = uint8_t(RobotMotionState::JointDamping);
            std::cout << "[MODE] Joint Damping\n";
        }
        else if (k == 'z' && msfb_->GetCurrentState() == RobotMotionState::WaitingForStand) {
            usr_cmd_->target_mode = uint8_t(RobotMotionState::StandingUp);
            std::cout << "[MODE] Standing Up\n";
        }
        else if (k == 'c' && msfb_->GetCurrentState() == RobotMotionState::StandingUp) {
            usr_cmd_->target_mode = uint8_t(RobotMotionState::RLControlMode);
            std::cout << "[MODE] RL Control\n";
        }
    }

    void keyboard_loop()
    {
        setup_raw_mode();

        std::cout << "\n╔════════════════════════════════════════════════╗\n"
                  << "║      KEYBOARD TELEOP - MULTI-KEY READY         ║\n"
                  << "╚════════════════════════════════════════════════╝\n"
                  << "  Movement:  W/S (forward/back)  A/D (left/right)\n"
                  << "  Rotation:  Q (CCW)  E (CW)\n"
                  << "  Mode:      R (damping)  Z (stand)  C (control)\n"
                  << "\n";

        char ch;

        while (running_) {
            double now = GetCurrentTimeStamp();
            usr_cmd_->time_stamp = now;

            // Read all available keyboard input
            while (read(STDIN_FILENO, &ch, 1) == 1) {
                char k = std::tolower(static_cast<unsigned char>(ch));

                // Handle mode commands
                if (k == 'r' || k == 'z' || k == 'c') {
                    process_mode_command(k);
                    continue;
                }

                // Track velocity keys
                if (velocity_keys_.count(k)) {
                    std::lock_guard<std::mutex> lock(keys_mutex_);
                    held_keys_.insert(k);
                    last_seen_time_[k] = now;
                }
            }

            // Remove keys that haven't been seen recently (released)
            {
                std::lock_guard<std::mutex> lock(keys_mutex_);
                std::vector<char> to_remove;
                
                for (char k : held_keys_) {
                    if (now - last_seen_time_[k] > key_timeout_ms_) {
                        to_remove.push_back(k);
                    }
                }
                
                for (char k : to_remove) {
                    held_keys_.erase(k);
                    last_seen_time_.erase(k);
                }
            }

            // Compute velocity from all currently held keys
            float fwd = 0.0f, side = 0.0f, yaw = 0.0f;
            
            if (msfb_->GetCurrentState() == RobotMotionState::RLControlMode) {
                compute_velocity_from_held_keys(fwd, side, yaw);
            }
            
            usr_cmd_->forward_vel_scale  = fwd;
            usr_cmd_->side_vel_scale     = side;
            usr_cmd_->turnning_vel_scale = yaw;

            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        restore_terminal();
        std::cout << "\n[KEYBOARD] Stopped.\n";
    }

public:
    KeyboardInterface(RobotName robot_name) : UserCommandInterface(robot_name)
    {
        std::cout << "[KeyboardInterface] Initialized with multi-key support\n";
        std::memset(usr_cmd_, 0, sizeof(UserCommand));
    }

    ~KeyboardInterface() 
    { 
        Stop(); 
    }

    void Start() override
    {
        if (running_) return;
        running_ = true;
        kb_thread_ = std::thread(&KeyboardInterface::keyboard_loop, this);
    }

    void Stop() override
    {
        running_ = false;
        if (kb_thread_.joinable()) {
            kb_thread_.join();
        }
        
        std::lock_guard<std::mutex> lock(keys_mutex_);
        held_keys_.clear();
        last_seen_time_.clear();
        
        usr_cmd_->forward_vel_scale = 0.0f;
        usr_cmd_->side_vel_scale = 0.0f;
        usr_cmd_->turnning_vel_scale = 0.0f;
    }

    UserCommand* GetUserCommand() override 
    { 
        return usr_cmd_; 
    }

    void set_max_velocities(float fwd, float side, float yaw)
    {
        max_forward_ = std::abs(fwd);
        max_side_    = std::abs(side);
        max_yaw_     = std::abs(yaw);
        std::cout << "[CONFIG] Max velocities: fwd=" << max_forward_ 
                  << " side=" << max_side_ 
                  << " yaw=" << max_yaw_ << "\n";
    }
};