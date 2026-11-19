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
 */
#pragma once

#include "common_types.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <string>

namespace fault {

#ifdef SDK_FAULT_INJECTION

struct Spec {
    std::string name;
    float start_s = 0.f;
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
        if (s.name != "policy_nan" && s.name != "policy_inf" && s.name != "policy_ramp") {
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
    if (s.name.empty()) return;
    const int start = int(s.start_s * policy_hz);
    if (run_cnt < start) return;
    if (run_cnt == start) std::fprintf(stderr, "[fault] injecting %s\n", s.name.c_str());
    if (s.name == "policy_nan") {
        action.setConstant(std::numeric_limits<float>::quiet_NaN());
    } else if (s.name == "policy_inf") {
        action.setConstant(std::numeric_limits<float>::infinity());
    } else if (s.name == "policy_ramp") {
        const float gain = std::pow(1.3f, float(run_cnt - start + 1));
        action = action.cwiseSign().cwiseMax(0.f) * 2.f - VecXf::Ones(action.size());  // +-1 pattern
        action *= gain;
    }
}

#else

inline void PolicyAction(VecXf&, int, float) {}

#endif

}  // namespace fault
