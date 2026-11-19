/**
 * @file policy_monitor.hpp
 * @brief Clips raw policy actions and decides when the policy must be stopped.
 *
 * Runs on every policy step, before the action is scaled into joint targets and long
 * before CommandGuard sees it. Both levels of stop put the robot in joint damping:
 *
 *   kSoftStop  the policy is misbehaving: damping for 3 s, then idle (like pressing 'r');
 *              the operator can stand up again without restarting.
 *   kHardStop  the robot is tilted too far: damping, latched until rl_deploy restarts.
 *
 * Clip and stop limits are per joint group (hip-x, hip-y, knee, wheel), because each group
 * has a different action scale and a different normal range.
 *
 * Soft-stop triggers: a non-finite action, any |raw action| above its group's raw limit,
 * the clip active on clip_steps_limit consecutive steps, or tilt above soft_tilt_deg.
 * Hard-stop trigger: tilt above hard_tilt_deg. Stops latch until Reset().
 *
 * All limits can be overridden from the environment (see Config::FromEnv), so they can be
 * tuned without rebuilding. Values outside sane ranges are rejected with a warning.
 */
#pragma once

#include "common_types.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

class PolicyMonitor {
public:
    using VecXi = Eigen::VectorXi;
    using VecXf = Eigen::VectorXf;
    using Vec3f = Eigen::Vector3f;
    enum Level { kOk = 0, kSoftStop = 1, kHardStop = 2 };

    enum Group { kHipX = 0, kHipY = 1, kKnee = 2, kWheel = 3, kNumGroups = 4 };
    static constexpr const char* kGroupEnv[kNumGroups] = {"HIPX", "HIPY", "KNEE", "WHEEL"};

    struct Config {
        // Per group: hip-x, hip-y, knee, wheel. Set from keyboard driving in sim with
        // m20_policy.onnx and all action limits off (2026-11-19, 40 s: forward/back, sideways,
        // turns, quick reversals). Observed max |raw|: hip-x 5.73, hip-y 2.27, knee 3.81,
        // wheel 7.08 (forward + turn). Clip ~1.4x and stop ~1.7x those. Re-measure on hardware.
        float clip[kNumGroups] = {8.f, 3.5f, 5.5f, 10.f};       // |raw action| bound
        float raw_limit[kNumGroups] = {10.f, 4.5f, 6.5f, 12.f};  // |raw action| above -> soft stop
        int clip_steps_limit = 10;    // clip active this many steps in a row (0.2 s) -> soft stop
        float soft_tilt_deg = 30.f;   // tilt above this -> soft stop (damping)
        float hard_tilt_deg = 45.f;   // tilt above this -> hard stop (damping, latched)

        void SetAllClips(float v) { for (float& c : clip) c = v; }
        void SetAllRawLimits(float v) { for (float& r : raw_limit) r = v; }

        // SDK_ACTION_CLIP sets every group's clip, SDK_ACTION_CLIP_LEG the three leg groups,
        // SDK_ACTION_CLIP_<HIPX|HIPY|KNEE|WHEEL> one group (later ones win). Same pattern for
        // SDK_SOFTSTOP_RAW[_LEG|_<GROUP>]. Also SDK_SOFTSTOP_CLIP_STEPS, SDK_SOFTSTOP_TILT_DEG,
        // SDK_HARDSTOP_TILT_DEG.
        static Config FromEnv() {
            Config c;
            ReadGroups("SDK_ACTION_CLIP", c.clip, 0.2f, 20.f);
            ReadGroups("SDK_SOFTSTOP_RAW", c.raw_limit, 0.5f, 30.f);
            float steps = float(c.clip_steps_limit);
            Read("SDK_SOFTSTOP_CLIP_STEPS", steps, 1.f, 500.f);
            c.clip_steps_limit = int(steps);
            Read("SDK_SOFTSTOP_TILT_DEG", c.soft_tilt_deg, 5.f, 80.f);
            Read("SDK_HARDSTOP_TILT_DEG", c.hard_tilt_deg, 5.f, 90.f);
            if (c.hard_tilt_deg < c.soft_tilt_deg) c.hard_tilt_deg = c.soft_tilt_deg;
            return c;
        }

    private:
        static void ReadGroups(const std::string& base, float (&v)[kNumGroups], float lo, float hi) {
            float all = NAN;
            if (Read(base.c_str(), all, lo, hi)) for (float& x : v) x = all;
            float leg = NAN;
            if (Read((base + "_LEG").c_str(), leg, lo, hi)) v[kHipX] = v[kHipY] = v[kKnee] = leg;
            for (int g = 0; g < kNumGroups; ++g) Read((base + "_" + kGroupEnv[g]).c_str(), v[g], lo, hi);
        }

        static bool Read(const char* name, float& value, float lo, float hi) {
            const char* env = std::getenv(name);
            if (!env || !*env) return false;
            char* end = nullptr;
            const float v = std::strtof(env, &end);
            if (end == env || *end != '\0' || !std::isfinite(v) || v < lo || v > hi) {
                std::fprintf(stderr, "[PolicyMonitor] ignoring %s=%s (allowed %.1f..%.1f)\n", name, env, lo, hi);
                return false;
            }
            value = v;
            return true;
        }
    };

    // Group of action i in policy order: 12 leg joints as (hipx, hipy, knee) per leg,
    // then the wheels.
    static Group GroupOf(int i, int num_leg_joints = 12) {
        return i < num_leg_joints ? Group(i % 3) : kWheel;
    }

    PolicyMonitor(const Config& cfg, int action_dim = 16, int num_leg_joints = 12)
        : cfg_(cfg), clip_(action_dim), raw_limit_(action_dim), raw_abs_max_(VecXf::Zero(action_dim)),
          clip_count_(VecXi::Zero(action_dim)) {
        for (int i = 0; i < action_dim; ++i) {
            clip_(i) = cfg.clip[GroupOf(i, num_leg_joints)];
            raw_limit_(i) = cfg.raw_limit[GroupOf(i, num_leg_joints)];
        }
    }

    void Reset() {
        level_ = kOk;
        reason_.clear();
        consecutive_clipped_ = 0;
        clipped_steps_ = 0;
        steps_ = 0;
        raw_abs_max_.setZero();
        clip_count_.setZero();
    }

    // Tilt from projected gravity (body frame). Upright is (0, 0, -1).
    static float TiltDeg(const Vec3f& projected_gravity) {
        const float c = std::fmax(-1.f, std::fmin(1.f, -projected_gravity.z() / projected_gravity.norm()));
        return std::acos(c) * 180.f / float(M_PI);
    }

    // Checks posture before inference. Returns the (latched) stop level.
    Level CheckPosture(float tilt_deg) {
        if (level_ == kHardStop) return level_;
        if (!std::isfinite(tilt_deg) || tilt_deg > cfg_.hard_tilt_deg)
            Stop(kHardStop, Fmt("tilt %.1f deg > %.0f", tilt_deg, cfg_.hard_tilt_deg));
        else if (tilt_deg > cfg_.soft_tilt_deg)
            Stop(kSoftStop, Fmt("tilt %.1f deg > %.0f", tilt_deg, cfg_.soft_tilt_deg));
        return level_;
    }

    // Records stats, checks, and clips the raw action in place. A stopped step leaves
    // the action untouched; the caller must not use it.
    Level CheckAndClip(VecXf& action, const std::vector<std::string>* names = nullptr) {
        if (level_ != kOk) return level_;
        ++steps_;
        for (int i = 0; i < action.size(); ++i) {
            if (!std::isfinite(action(i))) {
                Stop(kSoftStop, Fmt("non-finite action on %s", Name(names, i).c_str()));
                return level_;
            }
        }
        VecXf mag = action.cwiseAbs();
        raw_abs_max_ = raw_abs_max_.cwiseMax(mag);
        for (int i = 0; i < action.size(); ++i) {
            if (mag(i) > raw_limit_(i)) {
                Stop(kSoftStop, Fmt("|raw action| %.2f on %s > %.1f", mag(i), Name(names, i).c_str(), raw_limit_(i)));
                return level_;
            }
        }
        bool clipped = false;
        for (int i = 0; i < action.size(); ++i) {
            if (mag(i) > clip_(i)) {
                action(i) = std::copysign(clip_(i), action(i));
                ++clip_count_(i);
                clipped = true;
            }
        }
        if (clipped) {
            ++clipped_steps_;
            if (++consecutive_clipped_ >= cfg_.clip_steps_limit) {
                Stop(kSoftStop, Fmt("action clipped %d steps in a row", consecutive_clipped_));
                return level_;
            }
        } else {
            consecutive_clipped_ = 0;
        }
        return level_;
    }

    Level level() const { return level_; }
    const std::string& reason() const { return reason_; }
    const Config& config() const { return cfg_; }
    int steps() const { return steps_; }
    int clipped_steps() const { return clipped_steps_; }
    const VecXf& raw_abs_max() const { return raw_abs_max_; }
    const VecXi& clip_count() const { return clip_count_; }

private:
    void Stop(Level level, const std::string& why) {
        if (level <= level_) return;
        level_ = level;
        reason_ = why;
        std::printf("[PolicyMonitor] %s: %s\n", level == kHardStop ? "HARD STOP (damping, latched)" : "SOFT STOP (damping)",
                    why.c_str());
        std::fflush(stdout);
    }

    template <typename... A>
    static std::string Fmt(const char* f, A... a) {
        char buf[160];
        std::snprintf(buf, sizeof(buf), f, a...);
        return buf;
    }

    static std::string Name(const std::vector<std::string>* names, int i) {
        return names && i < int(names->size()) ? (*names)[i] : "joint " + std::to_string(i);
    }

    Config cfg_;
    VecXf clip_, raw_limit_;
    Level level_ = kOk;
    std::string reason_;
    int consecutive_clipped_ = 0, clipped_steps_ = 0, steps_ = 0;
    VecXf raw_abs_max_;
    VecXi clip_count_;
};
