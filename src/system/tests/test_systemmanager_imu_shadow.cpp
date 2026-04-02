/**
 * @file test_systemmanager_imu_shadow.cpp
 * @brief Runtime tests for SystemManager IMU shadow mode.
 */

#include "app/AppConfig.hpp"
#include "common/Logger.hpp"
#include "sensors/ICamera.hpp"
#include "system/SystemManager.hpp"
#include "src/tests/support/test_common.hpp"

#include <chrono>
#include <memory>
#include <thread>
#include <utility>

using solar::FrameEvent;
using solar::ICamera;
using solar::Logger;
using solar::PixelFormat;
using solar::SystemManager;
using solar::TrackerState;
using solar::app::AppConfig;
using solar::app::CameraBackend;
using solar::app::ImuBackend;
using solar::app::ImuFeedbackMode;
using solar::app::ManualInputBackend;

namespace {

class FakeCamera final : public ICamera {
public:
    void registerFrameCallback(FrameCallback cb) override {
        cb_ = std::move(cb);
    }

    bool start() override {
        started_ = true;
        return start_ok_;
    }

    void stop() override {
        started_ = false;
    }

    bool isRunning() const override {
        return started_;
    }

    void setStartResult(bool ok) {
        start_ok_ = ok;
    }

private:
    bool started_{false};
    bool start_ok_{true};
    FrameCallback cb_{};
};

AppConfig makeShadowConfig() {
    AppConfig cfg = solar::app::defaultConfig();
    cfg.camera_backend = CameraBackend::Simulated;
    cfg.manual_input_backend = ManualInputBackend::None;
    cfg.imu_backend = ImuBackend::None;
    cfg.imu_feedback_mode = ImuFeedbackMode::Shadow;
    cfg.servo.startup_policy = solar::ServoDriver::StartupPolicy::LogOnly;
    cfg.startup_mode = solar::app::StartupMode::Auto;
    return cfg;
}

} // namespace

TEST_CASE(SystemManager_ImuShadowMode_StartsWithoutImuBackendWhenDisabled) {
    Logger log;
    auto cam = std::make_unique<FakeCamera>();
    auto cfg = makeShadowConfig();

    SystemManager system(log, std::move(cam), cfg);

    REQUIRE(system.start());
    REQUIRE(system.state() == TrackerState::SEARCHING || system.state() == TrackerState::MANUAL);

    system.stop();
    REQUIRE(system.state() == TrackerState::IDLE);
}

TEST_CASE(SystemManager_ImuShadowMode_DoesNotPreventManualModeTransitions) {
    Logger log;
    auto cam = std::make_unique<FakeCamera>();
    auto cfg = makeShadowConfig();

    SystemManager system(log, std::move(cam), cfg);

    REQUIRE(system.start());

    system.enterManual();
    REQUIRE(system.state() == TrackerState::MANUAL);

    system.exitManual();
    REQUIRE(system.state() == TrackerState::SEARCHING);

    system.stop();
}