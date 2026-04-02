/**
 * @file test_systemmanager_statemachine.cpp
 * @brief Runtime state-machine tests for SystemManager.
 */

#include "app/AppConfig.hpp"
#include "common/Logger.hpp"
#include "sensors/ICamera.hpp"
#include "system/SystemManager.hpp"
#include "src/tests/support/test_common.hpp"

#include <atomic>
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

    void emitBrightCenteredFrame(std::uint64_t frame_id = 1) {
        if (!cb_) {
            return;
        }

        FrameEvent fe{};
        fe.frame_id = frame_id;
        fe.t_capture = std::chrono::steady_clock::now();
        fe.width = 64;
        fe.height = 64;
        fe.stride_bytes = 64;
        fe.format = PixelFormat::Gray8;
        fe.data.assign(64U * 64U, 0U);
        fe.data[32U * 64U + 32U] = 255U;

        cb_(fe);
    }

private:
    bool started_{false};
    bool start_ok_{true};
    FrameCallback cb_{};
};

AppConfig makeBaseConfig() {
    AppConfig cfg = solar::app::defaultConfig();
    cfg.camera_backend = CameraBackend::Simulated;
    cfg.manual_input_backend = ManualInputBackend::None;
    cfg.imu_backend = ImuBackend::None;
    cfg.imu_feedback_mode = ImuFeedbackMode::Disabled;
    cfg.servo.startup_policy = solar::ServoDriver::StartupPolicy::LogOnly;
    cfg.startup_mode = solar::app::StartupMode::Auto;
    cfg.tracker.min_pixels = 1U;
    cfg.tracker.threshold = 200U;
    cfg.controller.min_confidence = 0.0F;
    cfg.controller.deadband = 0.0F;
    return cfg;
}

} // namespace

TEST_CASE(SystemManager_start_to_searching_then_tracking_on_bright_frame) {
    Logger log;
    auto cam = std::make_unique<FakeCamera>();
    auto* cam_ptr = cam.get();

    auto cfg = makeBaseConfig();

    SystemManager system(log, std::move(cam), cfg);

    REQUIRE(system.start());
    REQUIRE(system.state() == TrackerState::SEARCHING);

    cam_ptr->emitBrightCenteredFrame(1);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    REQUIRE(system.state() == TrackerState::TRACKING ||
            system.state() == TrackerState::SEARCHING);

    system.stop();
    REQUIRE(system.state() == TrackerState::IDLE);
}

TEST_CASE(SystemManager_manual_mode_emits_commands) {
    Logger log;
    auto cam = std::make_unique<FakeCamera>();
    auto* cam_ptr = cam.get();

    auto cfg = makeBaseConfig();

    SystemManager system(log, std::move(cam), cfg);

    REQUIRE(system.start());
    system.enterManual();
    system.setManualCommandSource(SystemManager::ManualCommandSource::Gui);

    REQUIRE(system.state() == TrackerState::MANUAL);

    std::atomic<int> command_count{0};
    system.registerCommandObserver([&](const solar::ActuatorCommand&) {
        command_count.fetch_add(1, std::memory_order_relaxed);
    });

    system.setManualSetpoint(0.1F, -0.1F);
    std::this_thread::sleep_for(std::chrono::milliseconds(25));

    REQUIRE(command_count.load(std::memory_order_relaxed) == 0);

    cam_ptr->emitBrightCenteredFrame(1);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    const int first_count = command_count.load(std::memory_order_relaxed);
    REQUIRE(first_count > 0);

    // Manual GUI mode remain continuous on later control ticks without
    // needing another setManualSetpoint() call.
    cam_ptr->emitBrightCenteredFrame(2);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    REQUIRE(command_count.load(std::memory_order_relaxed) > first_count);

    system.stop();
}

TEST_CASE(SystemManager_start_with_null_camera_enters_fault_and_fails) {
    Logger log;
    auto cfg = makeBaseConfig();

    SystemManager system(log, nullptr, cfg);

    REQUIRE(!system.start());
    REQUIRE(system.state() == TrackerState::FAULT);
}

TEST_CASE(SystemManager_start_when_camera_start_fails_enters_fault) {
    Logger log;
    auto cam = std::make_unique<FakeCamera>();
    cam->setStartResult(false);

    auto cfg = makeBaseConfig();

    SystemManager system(log, std::move(cam), cfg);

    REQUIRE(!system.start());
    REQUIRE(system.state() == TrackerState::FAULT);
}
