/**
 * @file fault_injection.hpp
 * @brief Test-only faults for exercising the safety layer in simulation.
 *
 * Compiled in only when SDK_FAULT_INJECTION is defined. CMake defines it by default for
 * x86 (simulation) builds and never for the robot's arm build. Without the define, every
 * function here is an empty inline and the environment variable is ignored.
 *
 * Select a fault with SDK_FAULT_INJECT="<fault>@<seconds after RL control starts>":
 *   policy_nan@3    policy outputs NaN
 *   policy_inf@3    policy outputs +inf
 *   policy_ramp@3   policy outputs grow 1.3x per step, like the divergence seen in sim
 *   tilt35@3        PolicyMonitor sees a 35 deg tilt (any whole number of degrees). Only the
 *                   monitor's reading is faked; the robot itself is upright.
 *   noise1@3        Gaussian noise with std 1 (raw policy-output units) added to every
 *                   action, every step (any value, e.g. noise0.5)
 *   noiseramp0.5@3  same, but the std grows from 0 by 0.5 per second
 * Noise uses a fixed seed (0) so runs repeat; set SDK_FAULT_SEED to change it.
 */
#pragma once

#include "common_types.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <random>
#include <string>

namespace fault {

#ifdef SDK_FAULT_INJECTION

struct Spec {
    std::string name;
    float start_s = 0.f;
    float tilt_deg = 0.f;  // for tilt<N>
    float noise_std = 0.f;   // for noise<std>
    float noise_rate = 0.f;  // for noiseramp<rate>, std growth per second
};

inline const Spec& ActiveSpec() {
    static const Spec spec = [] {
        Spec s;
        const char* env = std::getenv("SDK_FAULT_INJECT");
        if (!env || !*env) return s;
        std::string v(env);
        const auto at = v.find('@');
        s.name = v.substr(0, at);
        if (at != std::string::npos) s.start_s = std::strtof(v.c_str() + at + 1, nullptr);
        if (s.name.rfind("tilt", 0) == 0 && s.name.size() > 4) s.tilt_deg = std::strtof(s.name.c_str() + 4, nullptr);
        if (s.name.rfind("noiseramp", 0) == 0 && s.name.size() > 9)
            s.noise_rate = std::strtof(s.name.c_str() + 9, nullptr);
        else if (s.name.rfind("noise", 0) == 0 && s.name.size() > 5)
            s.noise_std = std::strtof(s.name.c_str() + 5, nullptr);
        if (s.name != "policy_nan" && s.name != "policy_inf" && s.name != "policy_ramp" && s.tilt_deg <= 0.f &&
            s.noise_std <= 0.f && s.noise_rate <= 0.f) {
            std::fprintf(stderr, "[fault] unknown SDK_FAULT_INJECT '%s', ignoring\n", env);
            s.name.clear();
        } else {
            std::fprintf(stderr, "[fault] armed: %s at %.2f s after RL control starts\n", s.name.c_str(), s.start_s);
        }
        return s;
    }();
    return spec;
}

// run_cnt: policy steps since RL control started; policy_hz: policy rate.
inline void PolicyAction(VecXf& action, int run_cnt, float policy_hz) {
    const Spec& s = ActiveSpec();
    if (s.name.empty() || s.tilt_deg > 0.f) return;
    const int start = int(s.start_s * policy_hz);
    if (run_cnt < start) return;
    if (run_cnt == start) std::fprintf(stderr, "[fault] injecting %s\n", s.name.c_str());
    if (s.name == "policy_nan") {
        action.setConstant(std::numeric_limits<float>::quiet_NaN());
    } else if (s.name == "policy_inf") {
        action.setConstant(std::numeric_limits<float>::infinity());
    } else if (s.noise_std > 0.f || s.noise_rate > 0.f) {
        static std::mt19937 rng([] {
            const char* seed = std::getenv("SDK_FAULT_SEED");
            return seed ? unsigned(std::strtoul(seed, nullptr, 10)) : 0u;
        }());
        const float std_now = s.noise_std > 0.f ? s.noise_std : s.noise_rate * float(run_cnt - start + 1) / policy_hz;
        std::normal_distribution<float> n(0.f, 1.f);
        for (int i = 0; i < action.size(); ++i) action(i) += std_now * n(rng);
        const int every = int(policy_hz);  // print the current std once a second for ramps
        if (s.noise_rate > 0.f && (run_cnt - start) % every == 0)
            std::fprintf(stderr, "[fault] noise std now %.2f\n", std_now);
    } else if (s.name == "policy_ramp") {
        const float gain = std::pow(1.3f, float(run_cnt - start + 1));
        action = action.cwiseSign().cwiseMax(0.f) * 2.f - VecXf::Ones(action.size());  // +-1 pattern
        action *= gain;
    }
}

// Replaces the tilt PolicyMonitor sees.
inline void Tilt(float& tilt_deg, int run_cnt, float policy_hz) {
    const Spec& s = ActiveSpec();
    if (s.tilt_deg <= 0.f) return;
    const int start = int(s.start_s * policy_hz);
    if (run_cnt < start) return;
    if (run_cnt == start) std::fprintf(stderr, "[fault] injecting %s\n", s.name.c_str());
    tilt_deg = s.tilt_deg;
}

#else

inline void PolicyAction(VecXf&, int, float) {}
inline void Tilt(float&, int, float) {}

#endif

}  // namespace fault
