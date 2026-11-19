/**
 * @file command_guard.hpp
 * @brief Last check on every joint command before it is published to the motors.
 *
 * The SDK publishes /JOINTS_CMD straight to the motor drivers, so this is the only
 * place a bad command can be stopped. Every state's SetJointCommand() goes through
 * CommandGuard::Filter().
 *
 * Command layout: one row per joint in robot order (per leg: hipx, hipy, knee, wheel;
 * legs FL, FR, HL, HR), columns [kp, pos, kd, vel, tau_ff], in the controller's joint
 * frame (the same frame as RobotInterface::GetJointPosition()).
 *
 * Two responses:
 *  - Clamp: a value modestly outside its limit is clamped to the limit and counted.
 *    Clamps are reported at most once a second.
 *  - Trip: a non-finite value, a negative gain, a wrong-sized command, or a gross
 *    violation (a sign the controller has diverged). The command is replaced with
 *    joint damping, the trip callback asks the state machine for damping, and every
 *    later command is replaced with damping until the process restarts.
 *
 * Trip() is public so other checks (for example fall detection) can use the same path.
 */
#pragma once

#include "common_types.h"
#include "parameters/control_parameters.h"

#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <mutex>
#include <sstream>
#include <string>

class CommandGuard {
public:
    using CmdMat = Eigen::Matrix<float, Eigen::Dynamic, 5>;
    enum Column { kKp = 0, kPos = 1, kKd = 2, kVel = 3, kTau = 4 };

    struct Limits {
        VecXf pos_lower, pos_upper;  // rad; +-inf for continuous joints (wheels)
        VecXf vel_limit;             // |velocity target|, rad/s
        VecXf tau_limit;             // |feed-forward torque|, Nm
        VecXf kp_max, kd_max;
        VecXf damping_kd;            // gains used for the damping command after a trip

        float pos_trip_margin = 0.5f;  // rad beyond a position limit => trip
        float vel_trip_factor = 1.5f;  // |vel| > factor * limit => trip
        float tau_trip_factor = 2.0f;  // |tau_ff| > factor * limit => trip
    };

    // Wheel motor torque limit from the M20 MJCF (actuatorfrcrange); ControlParameters
    // has no wheel torque entry.
    static constexpr float kM20WheelTorqueLimit = 21.6f;
    // Largest gains any M20 state uses: lie-down kp 300 / kd 4.5 legs, kd 3 wheels.
    static constexpr float kM20LegKpMax = 300.f, kM20LegKdMax = 10.f, kM20WheelKdMax = 5.f;

    static Limits M20Limits(const ControlParameters& cp) {
        const int dof = 16;
        const float inf = std::numeric_limits<float>::infinity();
        Limits l;
        l.pos_lower.resize(dof); l.pos_upper.resize(dof);
        l.vel_limit.resize(dof); l.tau_limit.resize(dof);
        l.kp_max.resize(dof); l.kd_max.resize(dof); l.damping_kd.resize(dof);
        for (int leg = 0; leg < 4; ++leg) {
            const bool right = (leg == 1 || leg == 3);  // FR, HR: hipx range is mirrored
            for (int j = 0; j < 4; ++j) {
                const int i = leg * 4 + j;
                if (j == 3) {  // wheel: continuous, velocity-controlled
                    l.pos_lower(i) = -inf; l.pos_upper(i) = inf;
                    l.vel_limit(i) = cp.wheel_vel_limit_;
                    l.tau_limit(i) = kM20WheelTorqueLimit;
                    l.kp_max(i) = 0.f;
                    l.kd_max(i) = kM20WheelKdMax;
                    l.damping_kd(i) = 1.f;  // same as JointDampingState
                    continue;
                }
                if (j == 0 && right) {
                    l.pos_lower(i) = -cp.fl_joint_upper_(0);
                    l.pos_upper(i) = -cp.fl_joint_lower_(0);
                } else {
                    l.pos_lower(i) = cp.fl_joint_lower_(j);
                    l.pos_upper(i) = cp.fl_joint_upper_(j);
                }
                l.vel_limit(i) = cp.joint_vel_limit_(j);
                l.tau_limit(i) = cp.torque_limit_(j);
                l.kp_max(i) = kM20LegKpMax;
                l.kd_max(i) = kM20LegKdMax;
                l.damping_kd(i) = cp.swing_leg_kd_(j);
            }
        }
        return l;
    }

    explicit CommandGuard(const Limits& limits) : limits_(limits), dof_(int(limits.pos_lower.size())) {
        damping_ = CmdMat::Zero(dof_, 5);
        damping_.col(kKd) = limits_.damping_kd;
    }

    // Called once, outside the guard's lock, when the guard trips.
    void SetTripCallback(std::function<void(const std::string&)> cb) {
        std::lock_guard<std::mutex> lock(mutex_);
        trip_callback_ = std::move(cb);
    }

    // Checks and, where allowed, clamps cmd in place. Returns false if the guard is
    // tripped, in which case cmd has been replaced with the damping command.
    bool Filter(CmdMat& cmd) {
        std::string reason;
        bool tripped;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!tripped_) {
                reason = Check(cmd);
                if (!reason.empty()) Latch(reason);
            }
            tripped = tripped_;
            if (tripped) cmd = damping_;
            MaybeReportClamps();
        }
        if (!reason.empty()) Announce(reason);
        return !tripped;
    }

    void Trip(const std::string& reason) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (tripped_) return;
            Latch(reason);
        }
        Announce(reason);
    }

    bool Tripped() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return tripped_;
    }

    std::string TripReason() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return trip_reason_;
    }

    uint64_t ClampCount() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return total_clamps_;
    }

    const CmdMat& DampingCommand() const { return damping_; }

private:
    static const char* JointName(int i) {
        static const std::array<const char*, 16> names = {
            "fl_hipx", "fl_hipy", "fl_knee", "fl_wheel", "fr_hipx", "fr_hipy", "fr_knee", "fr_wheel",
            "hl_hipx", "hl_hipy", "hl_knee", "hl_wheel", "hr_hipx", "hr_hipy", "hr_knee", "hr_wheel"};
        return (i >= 0 && i < int(names.size())) ? names[i] : "joint?";
    }

    static const char* ColumnName(int c) {
        static const std::array<const char*, 5> names = {"kp", "pos", "kd", "vel", "tau_ff"};
        return names[c];
    }

    static std::string Describe(int joint, int col, float value, const std::string& what) {
        std::ostringstream ss;
        ss << JointName(joint) << " " << ColumnName(col) << " = " << value << " " << what;
        return ss.str();
    }

    // Returns a trip reason, or "" if the command is acceptable (possibly after clamping).
    std::string Check(CmdMat& cmd) {
        if (cmd.rows() != dof_) {
            return "command has " + std::to_string(cmd.rows()) + " joints, expected " + std::to_string(dof_);
        }
        for (int i = 0; i < dof_; ++i) {
            for (int c = 0; c < 5; ++c) {
                if (!std::isfinite(cmd(i, c))) return Describe(i, c, cmd(i, c), "(not finite)");
            }

            // Gains: negative is a bug, above the maximum is clamped.
            for (int c : {int(kKp), int(kKd)}) {
                if (cmd(i, c) < 0.f) return Describe(i, c, cmd(i, c), "(negative gain)");
            }
            Clamp(cmd(i, kKp), 0.f, limits_.kp_max(i), i, kKp);
            Clamp(cmd(i, kKd), 0.f, limits_.kd_max(i), i, kKd);

            // Position target.
            const float lo = limits_.pos_lower(i), hi = limits_.pos_upper(i);
            const float p = cmd(i, kPos);
            if (p > hi + limits_.pos_trip_margin || p < lo - limits_.pos_trip_margin) {
                std::ostringstream what;
                what << "(limit [" << lo << ", " << hi << "], more than " << limits_.pos_trip_margin << " rad outside)";
                return Describe(i, kPos, p, what.str());
            }
            Clamp(cmd(i, kPos), lo, hi, i, kPos);

            // Velocity target and feed-forward torque: symmetric limits.
            struct { int col; float limit, factor; } sym[] = {
                {kVel, limits_.vel_limit(i), limits_.vel_trip_factor},
                {kTau, limits_.tau_limit(i), limits_.tau_trip_factor}};
            for (const auto& s : sym) {
                const float v = cmd(i, s.col);
                if (std::fabs(v) > s.factor * s.limit) {
                    std::ostringstream what;
                    what << "(limit " << s.limit << ", more than " << s.factor << "x)";
                    return Describe(i, s.col, v, what.str());
                }
                Clamp(cmd(i, s.col), -s.limit, s.limit, i, s.col);
            }
        }
        return "";
    }

    void Clamp(float& v, float lo, float hi, int joint, int col) {
        float excess = 0.f;
        if (v > hi) { excess = v - hi; v = hi; }
        else if (v < lo) { excess = lo - v; v = lo; }
        else return;
        ++window_clamps_;
        ++total_clamps_;
        if (excess > worst_excess_) {
            worst_excess_ = excess;
            worst_joint_ = joint;
            worst_col_ = col;
        }
    }

    void MaybeReportClamps() {
        const auto now = std::chrono::steady_clock::now();
        if (window_clamps_ == 0) { window_start_ = now; return; }
        if (now - window_start_ < std::chrono::seconds(1)) return;
        std::printf("[CommandGuard] clamped %llu value(s) in the last second; largest overshoot %.3f on %s %s\n",
                    (unsigned long long)window_clamps_, worst_excess_, JointName(worst_joint_),
                    ColumnName(worst_col_));
        std::fflush(stdout);
        window_clamps_ = 0;
        worst_excess_ = 0.f;
        window_start_ = now;
    }

    void Latch(const std::string& reason) {
        tripped_ = true;
        trip_reason_ = reason;
    }

    void Announce(const std::string& reason) {
        std::fprintf(stderr, "\n[CommandGuard] TRIP: %s\n[CommandGuard] Sending joint damping until restart.\n\n",
                     reason.c_str());
        std::fflush(stderr);
        std::function<void(const std::string&)> cb;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            cb = trip_callback_;
        }
        if (cb) cb(reason);
    }

    const Limits limits_;
    const int dof_;
    CmdMat damping_;

    mutable std::mutex mutex_;
    bool tripped_ = false;
    std::string trip_reason_;
    std::function<void(const std::string&)> trip_callback_;

    uint64_t window_clamps_ = 0, total_clamps_ = 0;
    float worst_excess_ = 0.f;
    int worst_joint_ = 0, worst_col_ = 0;
    std::chrono::steady_clock::time_point window_start_ = std::chrono::steady_clock::now();
};
