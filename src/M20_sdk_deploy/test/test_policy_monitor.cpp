// Unit tests for PolicyMonitor (action clipping and soft/hard stop decisions).
// Build: colcon build; run: colcon test, or build/m20_sdk_deploy/test_policy_monitor.

#include "policy_monitor.hpp"

#include <cstdio>
#include <limits>

namespace {

int failures = 0;

#define EXPECT(cond)                                                          \
    do {                                                                      \
        if (!(cond)) {                                                        \
            std::printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);     \
            ++failures;                                                       \
        }                                                                     \
    } while (0)

using M = PolicyMonitor;
using VecXf = Eigen::VectorXf;
using Vec3f = Eigen::Vector3f;

// Fixed limits so the tests don't depend on the tuned defaults.
M::Config Cfg() {
    M::Config c;
    c.SetAllClips(2.f);
    c.SetAllRawLimits(3.f);
    c.clip_steps_limit = 5;
    c.soft_tilt_deg = 30.f;
    c.hard_tilt_deg = 45.f;
    return c;
}

Eigen::VectorXf Action(float v) { return VecXf::Constant(16, v); }

void TestNormalActionUnchanged() {
    std::printf("normal action passes unchanged\n");
    M m(Cfg());
    VecXf a = Action(1.05f), orig = a;
    EXPECT(m.CheckAndClip(a) == M::kOk);
    EXPECT(a.isApprox(orig));
    EXPECT(m.clipped_steps() == 0);
}

void TestClipPerGroup() {
    std::printf("legs and wheels clip at their own bounds\n");
    M::Config c = Cfg();
    c.clip[M::kHipX] = c.clip[M::kHipY] = c.clip[M::kKnee] = 1.f;
    c.clip[M::kWheel] = 2.5f;
    M m(c);
    VecXf a = Action(-2.4f);
    EXPECT(m.CheckAndClip(a) == M::kOk);
    EXPECT(a(0) == -1.f && a(11) == -1.f);      // legs
    EXPECT(a(12) == -2.4f && a(15) == -2.4f);   // wheels, under their clip
    EXPECT(m.clip_count()(0) == 1 && m.clip_count()(12) == 0);
}

void TestRawLimitSoftStops() {
    std::printf("|raw| above the raw limit soft-stops\n");
    M m(Cfg());
    VecXf a = Action(0.5f);
    a(4) = -3.5f;
    EXPECT(m.CheckAndClip(a) == M::kSoftStop);
    EXPECT(m.reason().find("joint 4") != std::string::npos);
}

void TestSustainedClippingSoftStops() {
    std::printf("clipping on consecutive steps soft-stops; a clean step resets the count\n");
    M m(Cfg());
    for (int i = 0; i < 4; ++i) { VecXf a = Action(2.5f); EXPECT(m.CheckAndClip(a) == M::kOk); }
    VecXf ok = Action(1.f);
    EXPECT(m.CheckAndClip(ok) == M::kOk);  // resets
    for (int i = 0; i < 4; ++i) { VecXf a = Action(2.5f); EXPECT(m.CheckAndClip(a) == M::kOk); }
    VecXf a = Action(2.5f);
    EXPECT(m.CheckAndClip(a) == M::kSoftStop);
}

void TestNonFiniteSoftStops() {
    std::printf("NaN and inf soft-stop\n");
    for (float bad : {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity()}) {
        M m(Cfg());
        VecXf a = Action(0.f);
        a(7) = bad;
        EXPECT(m.CheckAndClip(a) == M::kSoftStop);
    }
}

void TestTilt() {
    std::printf("tilt: soft above 30, hard above 45, hard overrides soft\n");
    EXPECT(std::fabs(M::TiltDeg(Vec3f(0, 0, -1))) < 1e-3f);
    EXPECT(std::fabs(M::TiltDeg(Vec3f(std::sin(0.6f), 0, -std::cos(0.6f))) - 34.38f) < 0.05f);
    M m(Cfg());
    EXPECT(m.CheckPosture(10.f) == M::kOk);
    EXPECT(m.CheckPosture(35.f) == M::kSoftStop);
    EXPECT(m.CheckPosture(20.f) == M::kSoftStop);  // latched
    EXPECT(m.CheckPosture(50.f) == M::kHardStop);  // escalates
    EXPECT(m.CheckPosture(10.f) == M::kHardStop);
    EXPECT(M(Cfg()).CheckPosture(std::numeric_limits<float>::quiet_NaN()) == M::kHardStop);
}

void TestLatchAndReset() {
    std::printf("a stop latches until Reset()\n");
    M m(Cfg());
    VecXf a = Action(5.f);
    EXPECT(m.CheckAndClip(a) == M::kSoftStop);
    VecXf b = Action(0.f);
    EXPECT(m.CheckAndClip(b) == M::kSoftStop);
    m.Reset();
    EXPECT(m.CheckAndClip(b) == M::kOk);
    EXPECT(m.reason().empty());
}

void TestEnvOverrides() {
    std::printf("env overrides apply; out-of-range values are ignored\n");
    setenv("SDK_ACTION_CLIP", "1", 1);
    setenv("SDK_ACTION_CLIP_WHEEL", "3", 1);
    setenv("SDK_SOFTSTOP_TILT_DEG", "999", 1);  // out of range
    M::Config c = M::Config::FromEnv();
    EXPECT(c.clip[M::kHipX] == 1.f && c.clip[M::kKnee] == 1.f);
    EXPECT(c.clip[M::kWheel] == 3.f);
    EXPECT(c.soft_tilt_deg == 30.f);
    unsetenv("SDK_ACTION_CLIP");
    unsetenv("SDK_ACTION_CLIP_WHEEL");
    unsetenv("SDK_SOFTSTOP_TILT_DEG");
}

void TestDefaultsPassMeasuredDriving() {
    std::printf("default limits: measured driving peaks pass, divergence-like hip-y stops\n");
    M m(M::Config{});
    VecXf a = Action(0.f);
    a(3) = 5.73f;   // fr_hipx, max seen when turning
    a(4) = 2.27f;   // fr_hipy
    a(8) = 3.81f;   // hl_knee
    a(12) = 7.08f;  // fl_wheel, forward + turn
    VecXf orig = a;
    EXPECT(m.CheckAndClip(a) == M::kOk);
    EXPECT(a.isApprox(orig));  // nothing clipped
    VecXf b = Action(0.f);
    b(10) = 5.2f;   // hr_hipy above its 4.5 stop, though hip-x would allow it
    EXPECT(m.CheckAndClip(b) == M::kSoftStop);
    EXPECT(m.reason().find("joint 10") != std::string::npos);
}

void TestGroupMapping() {
    std::printf("policy-order actions map to hipx/hipy/knee per leg, then wheels\n");
    EXPECT(M::GroupOf(0) == M::kHipX && M::GroupOf(1) == M::kHipY && M::GroupOf(2) == M::kKnee);
    EXPECT(M::GroupOf(9) == M::kHipX && M::GroupOf(11) == M::kKnee);
    EXPECT(M::GroupOf(12) == M::kWheel && M::GroupOf(15) == M::kWheel);
}

void TestEnvPerGroupRaw() {
    std::printf("SDK_SOFTSTOP_RAW_<GROUP> overrides one group after the global value\n");
    setenv("SDK_SOFTSTOP_RAW", "7", 1);
    setenv("SDK_SOFTSTOP_RAW_HIPY", "4", 1);
    M::Config c = M::Config::FromEnv();
    EXPECT(c.raw_limit[M::kHipX] == 7.f && c.raw_limit[M::kWheel] == 7.f);
    EXPECT(c.raw_limit[M::kHipY] == 4.f);
    unsetenv("SDK_SOFTSTOP_RAW");
    unsetenv("SDK_SOFTSTOP_RAW_HIPY");
}

}  // namespace

int main() {
    TestNormalActionUnchanged();
    TestClipPerGroup();
    TestRawLimitSoftStops();
    TestSustainedClippingSoftStops();
    TestNonFiniteSoftStops();
    TestTilt();
    TestLatchAndReset();
    TestEnvOverrides();
    TestDefaultsPassMeasuredDriving();
    TestGroupMapping();
    TestEnvPerGroupRaw();
    std::printf(failures ? "\n%d FAILURE(S)\n" : "\nall tests passed\n", failures);
    return failures ? 1 : 0;
}
