// keyboard_interface.hpp
// sudo apt install libevdev-dev
// sudo adduser $USER input
// newgrp input

#pragma once

#include "user_command_interface.h"
#include "custom_types.h"
#include <libevdev-1.0/libevdev/libevdev.h>
#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <thread>
#include <atomic>
#include <mutex>
#include <unordered_set>
#include <vector>
#include <string>
#include <iostream>
#include <chrono>

using namespace interface;
using namespace types;

class KeyboardInterface : public UserCommandInterface
{
private:
    std::atomic<bool> running_{false};
    std::thread kb_thread_;

    float max_forward_ = 0.7f;
    float max_side_    = 0.5f;
    float max_yaw_     = 0.7f;

    // Current velocity commands (ramp-up on repeat, zero on release)
    float fwd = 0.0f, side = 0.0f, yaw = 0.0f;

    std::unordered_set<int> pressed_keys_;
    mutable std::mutex keys_mutex_;

    struct EvDevKeyboard {
        int fd;
        struct libevdev* dev;
        std::string name;
    };
    std::vector<EvDevKeyboard> keyboards_;

    void ClipNumber(float& num, float low, float high)
    {
        if (num < low) num = low;
        if (num > high) num = high;
    }

    bool isKeyPressed(int keycode) const
    {
        std::lock_guard<std::mutex> lock(keys_mutex_);
        return pressed_keys_.count(keycode);
    }

    // Called on press (value=1) and repeat (value=2)
    void apply_key_direction(int keycode)
    {
        const float step = 0.1f;

        if (keycode == KEY_W)       fwd  +=  step * max_forward_;
        else if (keycode == KEY_S)  fwd  -=  step * max_forward_;
        else if (keycode == KEY_A)  side +=  step * max_side_;
        else if (keycode == KEY_D)  side -=  step * max_side_;
        else if (keycode == KEY_Q)  yaw  +=  step * max_yaw_;
        else if (keycode == KEY_E)  yaw  -=  step * max_yaw_;

        ClipNumber(fwd,  -max_forward_, max_forward_);
        ClipNumber(side, -max_side_,    max_side_);
        ClipNumber(yaw,  -max_yaw_,     max_yaw_);
    }

    // Called exactly on key release (value=0)
    void handle_key_release(int keycode)
    {
        if ((keycode == KEY_W || keycode == KEY_S) &&
            !isKeyPressed(KEY_W) && !isKeyPressed(KEY_S)) {
            fwd = 0.0f;
        }
        if ((keycode == KEY_A || keycode == KEY_D) &&
            !isKeyPressed(KEY_A) && !isKeyPressed(KEY_D)) {
            side = 0.0f;
        }
        if ((keycode == KEY_Q || keycode == KEY_E) &&
            !isKeyPressed(KEY_Q) && !isKeyPressed(KEY_E)) {
            yaw = 0.0f;
        }
    }

    void process_mode_key(int keycode)
    {
        if (keycode == KEY_R) {
            usr_cmd_->target_mode = uint8_t(RobotMotionState::JointDamping);
            std::cout << "[MODE] Joint Damping\n";
        }
        else if (keycode == KEY_Z && msfb_->GetCurrentState() == RobotMotionState::WaitingForStand) {
            usr_cmd_->target_mode = uint8_t(RobotMotionState::StandingUp);
            std::cout << "[MODE] Standing Up\n";
        }
        else if (keycode == KEY_C && msfb_->GetCurrentState() == RobotMotionState::StandingUp) {
            usr_cmd_->target_mode = uint8_t(RobotMotionState::RLControlMode);
            std::cout << "[MODE] RL Control\n";
        }
    }

    bool init_all_keyboards()
    {
        DIR* dir = opendir("/dev/input");
        if (!dir) return false;

        struct dirent* ent;
        while ((ent = readdir(dir)) != nullptr) {
            if (strncmp(ent->d_name, "event", 5) != 0) continue;

            std::string path = "/dev/input/" + std::string(ent->d_name);
            int fd = open(path.c_str(), O_RDONLY | O_NONBLOCK);
            if (fd < 0) continue;

            struct libevdev* dev = nullptr;
            if (libevdev_new_from_fd(fd, &dev) < 0) {
                close(fd);
                continue;
            }

            if (!libevdev_has_event_type(dev, EV_KEY) ||
                !libevdev_has_event_code(dev, EV_KEY, KEY_A)) {
                libevdev_free(dev);
                close(fd);
                continue;
            }

            const char* name = libevdev_get_name(dev);
            std::string name_str = name ? name : "Unknown";

            if (name_str.find("Mouse") != std::string::npos ||
                name_str.find("Touchpad") != std::string::npos) {
                libevdev_free(dev);
                close(fd);
                continue;
            }

            keyboards_.push_back({fd, dev, name_str});
            std::cout << "[KB] Detected: " << name_str << " → " << path << "\n";
        }
        closedir(dir);

        if (keyboards_.empty()) {
            std::cerr << "[KeyboardInterface] ERROR: No keyboards found!\n";
            return false;
        }

        std::cout << "[KeyboardInterface] Initialized " << keyboards_.size() << " keyboard(s).\n";
        return true;
    }

    void keyboard_loop()
    {
        if (!init_all_keyboards()) {
            std::cerr << "[KeyboardInterface] Failed to initialize keyboards!\n";
            return;
        }

        std::cout << "\n╔════════════════════════════════════════════════╗\n"
                  << "║       KEYBOARD TELEOP – RAMP-UP + INSTANT STOP ║\n"
                  << "╚════════════════════════════════════════════════╝\n"
                  << "  Hold W/S/A/D/Q/E → velocity ramps up\n"
                  << "  Release key      → instant stop on that axis\n"
                  << "  Modes: R (damping)  Z (stand)  C (RL control)\n\n";

        struct input_event ev;

        while (running_) {
            bool got_event = false;

            for (auto& kb : keyboards_) {
                while (libevdev_next_event(kb.dev, LIBEVDEV_READ_FLAG_NORMAL, &ev) == 0) {
                    got_event = true;
                    if (ev.type != EV_KEY) continue;

                    bool was_pressed = pressed_keys_.count(ev.code);

                    // Update pressed state
                    {
                        std::lock_guard<std::mutex> lock(keys_mutex_);
                        if (ev.value == 1 || ev.value == 2) {
                            pressed_keys_.insert(ev.code);
                        } else if (ev.value == 0) {
                            pressed_keys_.erase(ev.code);
                        }
                    }

                    // Press or repeat → ramp up
                    if (ev.value == 1 || ev.value == 2) {
                        if (ev.code == KEY_W || ev.code == KEY_S ||
                            ev.code == KEY_A || ev.code == KEY_D ||
                            ev.code == KEY_Q || ev.code == KEY_E) {
                            apply_key_direction(ev.code);
                        }
                    }

                    // Release → zero axis if needed
                    if (was_pressed && ev.value == 0) {
                        handle_key_release(ev.code);
                    }

                    // Mode keys (only on real press)
                    if (ev.value == 1) {
                        if (ev.code == KEY_R || ev.code == KEY_Z || ev.code == KEY_C) {
                            process_mode_key(ev.code);
                        }
                        if (ev.code == KEY_ESC) {
                            std::cout << "\n[KEYBOARD] ESC pressed → stopping.\n";
                            running_ = false;
                        }
                    }
                }
            }

            // ─────── CRITICAL: ALWAYS publish current values every loop ───────
            usr_cmd_->forward_vel_scale  = fwd;
            usr_cmd_->side_vel_scale     = side;
            usr_cmd_->turnning_vel_scale = yaw;
            usr_cmd_->time_stamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count() / 1000.0;

            // Optional nice live display
            if (msfb_->GetCurrentState() == RobotMotionState::RLControlMode) {
                std::cout << "\rvel: " << fwd << "  " << side << "  " << yaw << std::flush;
            }

            if (!got_event) {
                std::this_thread::sleep_for(std::chrono::milliseconds(4));
            }
        }
        std::cout << std::endl;
    }

public:
    KeyboardInterface(RobotName robot_name) : UserCommandInterface(robot_name)
    {
        std::cout << "[KeyboardInterface] Ramp-up + instant-stop version ready!\n";
        std::memset(usr_cmd_, 0, sizeof(UserCommand));
    }

    ~KeyboardInterface() { Stop(); }

    void Start() override
    {
        if (running_) return;
        running_ = true;
        kb_thread_ = std::thread(&KeyboardInterface::keyboard_loop, this);
    }

    void Stop() override
    {
        running_ = false;
        if (kb_thread_.joinable()) kb_thread_.join();

        for (auto& kb : keyboards_) {
            libevdev_free(kb.dev);
            close(kb.fd);
        }
        keyboards_.clear();

        usr_cmd_->forward_vel_scale = usr_cmd_->side_vel_scale = usr_cmd_->turnning_vel_scale = 0.0f;
    }

    UserCommand* GetUserCommand() override { return usr_cmd_; }

    void set_max_velocities(float fwd, float side, float yaw)
    {
        max_forward_ = std::abs(fwd);
        max_side_    = std::abs(side);
        max_yaw_     = std::abs(yaw);
        std::cout << "[CONFIG] Max velocities → fwd:" << max_forward_
                  << " side:" << max_side_
                  << " yaw:" << max_yaw_ << "\n";
    }
};