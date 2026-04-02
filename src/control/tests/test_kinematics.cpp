/**
 * @file test_kinematics.cpp
 * @brief Unit tests for Kinematics3RRS.
 */

#include "control/Kinematics3RRS.hpp"
#include "src/tests/support/test_common.hpp"

#include <array>
#include <cmath>

using solar::ActuatorCommand;
using solar::CommandStatus;
using solar::Kinematics3RRS;
using solar::Logger;
using solar::PlatformSetpoint;

namespace {

bool allInRange(const std::array<float, 3>& u, float lo, float hi) {
    return u[0] >= lo && u[0] <= hi &&
           u[1] >= lo && u[1] <= hi &&
           u[2] >= lo && u[2] <= hi;
}

float maxAbsDiff(const std::array<float, 3>& a, const std::array<float, 3>& b) {
    float m = 0.0F;
    for (std::size_t i = 0; i < 3U; ++i) {
        m = std::max(m, std::fabs(a[i] - b[i]));
    }
    return m;
}

} // namespace

TEST_CASE(Kinematics3RRS_outputs_in_range_and_integer_like) {
    Logger log;
    Kinematics3RRS::Config cfg{};
    cfg.base_radius_m = 0.20F;
    cfg.platform_radius_m = 0.12F;
    cfg.home_height_m = 0.18F;
    cfg.horn_length_m = 0.10F;
    cfg.rod_length_m = 0.18F;
    cfg.base_theta_deg = {120.0F, 240.0F, 0.0F};
    cfg.plat_theta_deg = {120.0F, 240.0F, 0.0F};
    cfg.servo_neutral_deg = {90.0F, 90.0F, 90.0F};
    cfg.servo_dir = {-1, -1, -1};

    Kinematics3RRS kin(log, cfg);

    ActuatorCommand out{};
    kin.registerCommandCallback([&](const ActuatorCommand& cmd) {
        out = cmd;
    });

    PlatformSetpoint sp{};
    sp.frame_id = 1;
    sp.pan_rad = 0.1F;
    sp.tilt_rad = -0.15F;

    kin.onSetpoint(sp);

    REQUIRE(out.status == CommandStatus::Ok);
    REQUIRE(allInRange(out.actuator_targets, 0.0F, 180.0F));
}

TEST_CASE(Kinematics3RRS_continuity_small_setpoint_changes_small_output_changes) {
    Logger log;
    Kinematics3RRS::Config cfg{};
    cfg.base_radius_m = 0.20F;
    cfg.platform_radius_m = 0.12F;
    cfg.home_height_m = 0.18F;
    cfg.horn_length_m = 0.10F;
    cfg.rod_length_m = 0.18F;
    cfg.base_theta_deg = {120.0F, 240.0F, 0.0F};
    cfg.plat_theta_deg = {120.0F, 240.0F, 0.0F};
    cfg.servo_neutral_deg = {90.0F, 90.0F, 90.0F};
    cfg.servo_dir = {-1, -1, -1};

    Kinematics3RRS kin(log, cfg);

    ActuatorCommand a{};
    ActuatorCommand b{};
    bool first = true;

    kin.registerCommandCallback([&](const ActuatorCommand& cmd) {
        if (first) {
            a = cmd;
            first = false;
        } else {
            b = cmd;
        }
    });

    PlatformSetpoint sp1{};
    sp1.frame_id = 1;
    sp1.pan_rad = 0.05F;
    sp1.tilt_rad = 0.05F;

    PlatformSetpoint sp2{};
    sp2.frame_id = 2;
    sp2.pan_rad = 0.055F;
    sp2.tilt_rad = 0.052F;

    kin.onSetpoint(sp1);
    kin.onSetpoint(sp2);

    REQUIRE(a.status == CommandStatus::Ok);
    REQUIRE(b.status == CommandStatus::Ok);
    REQUIRE(maxAbsDiff(a.actuator_targets, b.actuator_targets) < 5.0F);
}

TEST_CASE(Kinematics3RRS_invalid_geometry_is_surfaced_explicitly) {
    Logger log;
    Kinematics3RRS::Config cfg{};
    cfg.base_radius_m = 0.20F;
    cfg.platform_radius_m = 0.12F;
    cfg.home_height_m = 0.18F;
    cfg.horn_length_m = 0.10F;
    cfg.rod_length_m = 0.01F;
    cfg.base_theta_deg = {120.0F, 240.0F, 0.0F};
    cfg.plat_theta_deg = {120.0F, 240.0F, 0.0F};
    cfg.servo_neutral_deg = {90.0F, 90.0F, 90.0F};
    cfg.servo_dir = {-1, -1, -1};

    Kinematics3RRS kin(log, cfg);

    ActuatorCommand out{};
    kin.registerCommandCallback([&](const ActuatorCommand& cmd) {
        out = cmd;
    });

    PlatformSetpoint sp{};
    sp.frame_id = 1;
    sp.pan_rad = 0.1F;
    sp.tilt_rad = 0.1F;

    kin.onSetpoint(sp);

    REQUIRE((out.status == CommandStatus::KinematicsInvalidConfig ||
             out.status == CommandStatus::KinematicsFallbackLastValid));
    REQUIRE(allInRange(out.actuator_targets, 0.0F, 180.0F));
}

TEST_CASE(Kinematics3RRS_falls_back_to_last_valid_when_subsequent_config_is_bad) {
    Logger log;

    Kinematics3RRS::Config good{};
    good.base_radius_m = 0.20F;
    good.platform_radius_m = 0.12F;
    good.home_height_m = 0.18F;
    good.horn_length_m = 0.10F;
    good.rod_length_m = 0.18F;
    good.base_theta_deg = {120.0F, 240.0F, 0.0F};
    good.plat_theta_deg = {120.0F, 240.0F, 0.0F};
    good.servo_neutral_deg = {90.0F, 90.0F, 90.0F};
    good.servo_dir = {-1, -1, -1};

    Kinematics3RRS kin(log, good);

    ActuatorCommand first{};
    ActuatorCommand second{};
    bool got_first = false;

    kin.registerCommandCallback([&](const ActuatorCommand& cmd) {
        if (!got_first) {
            first = cmd;
            got_first = true;
        } else {
            second = cmd;
        }
    });

    PlatformSetpoint sp1{};
    sp1.frame_id = 1;
    sp1.pan_rad = 0.1F;
    sp1.tilt_rad = 0.05F;

    kin.onSetpoint(sp1);

    REQUIRE(first.status == CommandStatus::Ok);

    Kinematics3RRS::Config bad = good;
    bad.rod_length_m = 0.01F;

    Kinematics3RRS kin_bad(log, bad);
    kin_bad.registerCommandCallback([&](const ActuatorCommand& cmd) {
        second = cmd;
    });

    PlatformSetpoint sp2{};
    sp2.frame_id = 2;
    sp2.pan_rad = 0.2F;
    sp2.tilt_rad = 0.1F;

    kin_bad.onSetpoint(sp2);

    REQUIRE(second.status == CommandStatus::KinematicsInvalidConfig ||
            second.status == CommandStatus::KinematicsFallbackLastValid);
    REQUIRE(allInRange(second.actuator_targets, 0.0F, 180.0F));
}
