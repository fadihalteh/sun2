#include "actuators/ServoDriver.hpp"
#include "src/tests/support/test_common.hpp"

#include <array>
#include <memory>
#include <vector>

using solar::actuators::CommandStatus;
using solar::actuators::PCA9685;
using solar::actuators::ServoCommand;
using solar::actuators::ServoDriver;

namespace {

struct FakePCA9685 final : public PCA9685 {
    struct Write {
        int channel;
        float pulse_us;
    };

    bool started_ok{true};
    bool start_called{false};
    bool stop_called{false};
    std::vector<Write> writes;

    bool start() override {
        start_called = true;
        return started_ok;
    }

    void stop() override {
        stop_called = true;
    }

    bool setPulseUs(int channel, float pulse_us) override {
        writes.push_back({channel, pulse_us});
        return true;
    }

    bool isStarted() const override {
        return start_called && started_ok;
    }
};

ServoDriver::Config makeConfig() {
    ServoDriver::Config cfg{};
    cfg.mode = ServoDriver::Mode::LogOnly;
    cfg.require_hardware = false;
    cfg.prefer_hardware = false;
    cfg.park_on_start = true;
    cfg.park_on_stop = true;
    cfg.neutral_deg = {41.0F, 41.0F, 41.0F};
    cfg.min_deg = {0.0F, 0.0F, 0.0F};
    cfg.max_deg = {180.0F, 180.0F, 180.0F};
    cfg.invert = {false, false, false};
    cfg.log_every_n = 1;
    return cfg;
}

bool nearlyEqual(float a, float b, float eps = 1e-3F) {
    return std::fabs(a - b) <= eps;
}

} // namespace

TEST_CASE(ServoDriver_log_only_mode_starts_without_hardware) {
    auto cfg = makeConfig();
    cfg.mode = ServoDriver::Mode::LogOnly;
    cfg.require_hardware = false;
    cfg.prefer_hardware = false;

    ServoDriver drv{cfg};
    REQUIRE(drv.start());
    REQUIRE(!drv.isHardwareMode());
    drv.stop();
}

TEST_CASE(ServoDriver_require_hardware_fails_fast_when_unavailable) {
    auto cfg = makeConfig();
    cfg.mode = ServoDriver::Mode::Hardware;
    cfg.require_hardware = true;
    cfg.prefer_hardware = false;

    auto fake = std::make_shared<FakePCA9685>();
    fake->started_ok = false;

    ServoDriver drv{cfg, fake};
    REQUIRE(!drv.start());
    REQUIRE(!drv.isHardwareMode());
}

TEST_CASE(ServoDriver_prefer_hardware_falls_back_to_log_only_when_unavailable) {
    auto cfg = makeConfig();
    cfg.mode = ServoDriver::Mode::Hardware;
    cfg.require_hardware = false;
    cfg.prefer_hardware = true;

    auto fake = std::make_shared<FakePCA9685>();
    fake->started_ok = false;

    ServoDriver drv{cfg, fake};
    REQUIRE(drv.start());
    REQUIRE(!drv.isHardwareMode());
    drv.stop();
}

TEST_CASE(ServoDriver_require_hardware_with_injected_pca_enters_hardware_mode) {
    auto cfg = makeConfig();
    cfg.mode = ServoDriver::Mode::Hardware;
    cfg.require_hardware = true;
    cfg.prefer_hardware = false;

    auto fake = std::make_shared<FakePCA9685>();
    fake->started_ok = true;

    ServoDriver drv{cfg, fake};
    REQUIRE(drv.start());
    REQUIRE(drv.isHardwareMode());
    drv.stop();
}

TEST_CASE(ServoDriver_apply_while_stopped_writes_nothing) {
    auto cfg = makeConfig();
    cfg.mode = ServoDriver::Mode::Hardware;

    auto fake = std::make_shared<FakePCA9685>();
    ServoDriver drv{cfg, fake};

    ServoCommand cmd{};
    cmd.status = CommandStatus::Ok;
    cmd.u = {10.0F, 20.0F, 30.0F};

    drv.apply(cmd);
    REQUIRE(fake->writes.empty());
}

TEST_CASE(ServoDriver_start_parks_to_neutral_when_enabled) {
    auto cfg = makeConfig();
    cfg.mode = ServoDriver::Mode::Hardware;
    cfg.park_on_start = true;
    cfg.park_on_stop = false;

    auto fake = std::make_shared<FakePCA9685>();
    ServoDriver drv{cfg, fake};

    REQUIRE(drv.start());
    REQUIRE(fake->writes.size() == 3);
    drv.stop();
}

TEST_CASE(ServoDriver_stop_parks_to_neutral_when_enabled) {
    auto cfg = makeConfig();
    cfg.mode = ServoDriver::Mode::Hardware;
    cfg.park_on_start = false;
    cfg.park_on_stop = true;

    auto fake = std::make_shared<FakePCA9685>();
    ServoDriver drv{cfg, fake};

    REQUIRE(drv.start());
    fake->writes.clear();

    drv.stop();
    REQUIRE(fake->writes.size() == 3);
}

TEST_CASE(ServoDriver_apply_clamps_and_writes_channels) {
    auto cfg = makeConfig();
    cfg.mode = ServoDriver::Mode::Hardware;
    cfg.park_on_start = false;
    cfg.park_on_stop = false;
    cfg.min_deg = {20.0F, 20.0F, 20.0F};
    cfg.max_deg = {160.0F, 160.0F, 160.0F};

    auto fake = std::make_shared<FakePCA9685>();
    ServoDriver drv{cfg, fake};

    REQUIRE(drv.start());

    ServoCommand cmd{};
    cmd.status = CommandStatus::Ok;
    cmd.u = {-100.0F, 90.0F, 999.0F};

    drv.apply(cmd);

    REQUIRE(fake->writes.size() == 3);
}

TEST_CASE(ServoDriver_inverted_channel_maps_correctly) {
    auto cfg = makeConfig();
    cfg.mode = ServoDriver::Mode::Hardware;
    cfg.park_on_start = false;
    cfg.park_on_stop = false;
    cfg.invert = {false, true, false};

    auto fake = std::make_shared<FakePCA9685>();
    ServoDriver drv{cfg, fake};

    REQUIRE(drv.start());

    ServoCommand cmd{};
    cmd.status = CommandStatus::Ok;
    cmd.u = {30.0F, 50.0F, 70.0F};

    drv.apply(cmd);
    REQUIRE(fake->writes.size() == 3);
}