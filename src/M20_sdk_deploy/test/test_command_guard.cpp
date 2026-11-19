// Unit tests for CommandGuard. Build: colcon build; run: colcon test, or run
// build/m20_sdk_deploy/test_command_guard directly. Exits nonzero on failure.

#include "command_guard.hpp"

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

using CmdMat = CommandGuard::CmdMat;

ControlParameters& Params() {
    static ControlParameters cp(RobotName::M20);
    return cp;
}

CommandGuard MakeGuard() { return CommandGuard(CommandGuard::M20Limits(Params())); }

// A plausible standing command: legs in PD, wheels in velocity mode.
CmdMat StandCommand() {
    CmdMat c = CmdMat::Zero(16, 5);
    for (int leg = 0; leg < 4; ++leg) {
        const float sign = leg < 2 ? 1.f : -1.f;  // hind hipy/knee are mirrored
        c.row(leg * 4 + 0) << 80, 0.0f, 2, 0, 0;
        c.row(leg * 4 + 1) << 80, -0.6f * sign, 2, 0, 0;
        c.row(leg * 4 + 2) << 80, 1.2f * sign, 2, 0, 0;
        c.row(leg * 4 + 3) << 0, 0.0f, 0.6f, 1.5f, 0;
    }
    return c;
}

void TestValidCommandPassesUnchanged() {
    std::printf("valid command passes unchanged\n");
    auto g = MakeGuard();
    CmdMat c = StandCommand(), orig = c;
    EXPECT(g.Filter(c));
    EXPECT(c.isApprox(orig));
    EXPECT(!g.Tripped());
    EXPECT(g.ClampCount() == 0);
}

void TestSmallOvershootIsClamped() {
    std::printf("small position overshoot is clamped, not tripped\n");
    auto g = MakeGuard();
    CmdMat c = StandCommand();
    const float knee_hi = Params().fl_joint_upper_(2);
    c(2, CommandGuard::kPos) = knee_hi + 0.26f;  // e.g. lie-down's knee target (3.02 rad)
    EXPECT(g.Filter(c));
    EXPECT(!g.Tripped());
    EXPECT(std::fabs(c(2, CommandGuard::kPos) - knee_hi) < 1e-6f);
    EXPECT(g.ClampCount() == 1);
}

void TestHipxIsMirroredForRightLegs() {
    std::printf("hipx limits are mirrored for right legs\n");
    auto g = MakeGuard();
    CmdMat c = StandCommand();
    const float hipx_hi = Params().fl_joint_upper_(0);   // 0.611
    const float hipx_lo = Params().fl_joint_lower_(0);   // -0.436
    c(0, CommandGuard::kPos) = 0.5f;   // FL: inside [-0.436, 0.611]
    c(4, CommandGuard::kPos) = 0.5f;   // FR: range is [-0.611, 0.436], so clamped
    EXPECT(g.Filter(c));
    EXPECT(std::fabs(c(0, CommandGuard::kPos) - 0.5f) < 1e-6f);
    EXPECT(std::fabs(c(4, CommandGuard::kPos) - (-hipx_lo)) < 1e-6f);
    (void)hipx_hi;
}

void TestNanTripsAndLatchesDamping() {
    std::printf("NaN trips, calls back once, and latches damping\n");
    auto g = MakeGuard();
    int callbacks = 0;
    g.SetTripCallback([&](const std::string&) { ++callbacks; });
    CmdMat c = StandCommand();
    c(5, CommandGuard::kPos) = std::numeric_limits<float>::quiet_NaN();
    EXPECT(!g.Filter(c));
    EXPECT(g.Tripped());
    EXPECT(callbacks == 1);
    EXPECT(c.isApprox(g.DampingCommand()));
    EXPECT(c.col(CommandGuard::kKp).isZero());
    EXPECT(c.col(CommandGuard::kPos).isZero());
    EXPECT((c.col(CommandGuard::kKd).array() > 0.f).all());

    CmdMat next = StandCommand();  // a valid command after the trip is still overridden
    EXPECT(!g.Filter(next));
    EXPECT(next.isApprox(g.DampingCommand()));
    EXPECT(callbacks == 1);
}

void TestInfTorqueTrips() {
    std::printf("infinite feed-forward torque trips\n");
    auto g = MakeGuard();
    CmdMat c = StandCommand();
    c(1, CommandGuard::kTau) = std::numeric_limits<float>::infinity();
    EXPECT(!g.Filter(c));
    EXPECT(g.Tripped());
}

void TestGrossPositionTrips() {
    std::printf("position far outside its limit trips (divergence)\n");
    auto g = MakeGuard();
    CmdMat c = StandCommand();
    c(9, CommandGuard::kPos) = 200.f;  // like the diverged policy's targets
    EXPECT(!g.Filter(c));
    EXPECT(g.TripReason().find("hl_hipy") != std::string::npos);
}

void TestWheelVelocity() {
    std::printf("wheel velocity: clamp above limit, trip far above\n");
    const float lim = Params().wheel_vel_limit_;
    {
        auto g = MakeGuard();
        CmdMat c = StandCommand();
        c(3, CommandGuard::kVel) = 1.2f * lim;
        EXPECT(g.Filter(c));
        EXPECT(std::fabs(c(3, CommandGuard::kVel) - lim) < 1e-4f);
    }
    {
        auto g = MakeGuard();
        CmdMat c = StandCommand();
        c(7, CommandGuard::kVel) = -2.f * lim;
        EXPECT(!g.Filter(c));
    }
}

void TestGains() {
    std::printf("gains: negative trips, too high clamps, wheel kp forced to 0\n");
    {
        auto g = MakeGuard();
        CmdMat c = StandCommand();
        c(2, CommandGuard::kKd) = -1.f;
        EXPECT(!g.Filter(c));
    }
    {
        auto g = MakeGuard();
        CmdMat c = StandCommand();
        c(1, CommandGuard::kKp) = 1000.f;
        c(3, CommandGuard::kKp) = 50.f;  // wheel
        EXPECT(g.Filter(c));
        EXPECT(c(1, CommandGuard::kKp) == CommandGuard::kM20LegKpMax);
        EXPECT(c(3, CommandGuard::kKp) == 0.f);
    }
}

void TestWrongSizeTrips() {
    std::printf("wrong-sized command trips\n");
    auto g = MakeGuard();
    CmdMat c = CmdMat::Zero(12, 5);
    EXPECT(!g.Filter(c));
    EXPECT(c.rows() == 16);  // replaced with the full damping command
}

void TestExternalTrip() {
    std::printf("Trip() from another check latches damping\n");
    auto g = MakeGuard();
    int callbacks = 0;
    g.SetTripCallback([&](const std::string&) { ++callbacks; });
    g.Trip("test: robot fell");
    g.Trip("second trip is ignored");
    CmdMat c = StandCommand();
    EXPECT(!g.Filter(c));
    EXPECT(callbacks == 1);
    EXPECT(g.TripReason() == "test: robot fell");
}

}  // namespace

int main() {
    TestValidCommandPassesUnchanged();
    TestSmallOvershootIsClamped();
    TestHipxIsMirroredForRightLegs();
    TestNanTripsAndLatchesDamping();
    TestInfTorqueTrips();
    TestGrossPositionTrips();
    TestWheelVelocity();
    TestGains();
    TestWrongSizeTrips();
    TestExternalTrip();
    std::printf(failures ? "\n%d FAILURE(S)\n" : "\nall tests passed\n", failures);
    return failures ? 1 : 0;
}
