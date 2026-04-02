/**
 * @file test_mpu6050_publisher.cpp
 * @brief Unit tests for Mpu6050Publisher.
 */

#include "sensors/imu/Mpu6050Publisher.hpp"
#include "src/tests/support/FakeI2CDevice.hpp"
#include "src/tests/support/test_common.hpp"

using solar::IIMU;
using solar::IImuSample;
using solar::Mpu6050Config;
using solar::Mpu6050Publisher;
using solar::tests::FakeI2CDevice;

namespace {

class SampleCollector final : public IIMU::CallbackInterface {
public:
    void hasSample(const IImuSample& sample) override {
        sample_count++;
        last = sample;
    }

    int sample_count{0};
    IImuSample last{};
};

} // namespace

TEST_CASE(Mpu6050Publisher_StartFailsWithoutCallback) {
    FakeI2CDevice dev;
    Mpu6050Config cfg{};
    cfg.i2c.bus = 1;
    cfg.i2c.address = 0x68U;
    cfg.gpio_pin_no = 17;
    cfg.gpio_chip_no = 0;

    Mpu6050Publisher pub(dev, cfg);
    REQUIRE(!pub.start());
}

TEST_CASE(Mpu6050Publisher_StartFailsIfWhoAmIDoesNotMatch) {
    FakeI2CDevice dev;
    Mpu6050Config cfg{};
    cfg.i2c.bus = 1;
    cfg.i2c.address = 0x68U;
    cfg.gpio_pin_no = 17;
    cfg.gpio_chip_no = 0;

    dev.setRegister(cfg.who_am_i_reg, 0x00U);

    Mpu6050Publisher pub(dev, cfg);
    SampleCollector cb;
    pub.registerEventCallback(&cb);

    REQUIRE(!pub.start());
}

TEST_CASE(Mpu6050Publisher_StartInitialisesDevice_WhenWhoAmIMatches) {
    FakeI2CDevice dev;
    Mpu6050Config cfg{};
    cfg.i2c.bus = 1;
    cfg.i2c.address = 0x68U;
    cfg.gpio_pin_no = 17;
    cfg.gpio_chip_no = 0;

    dev.setRegister(cfg.who_am_i_reg, cfg.who_am_i_expected);
    dev.setRegister(cfg.int_status_reg, cfg.data_ready_mask);

    Mpu6050Publisher pub(dev, cfg);
    SampleCollector cb;
    pub.registerEventCallback(&cb);

    REQUIRE(pub.start());
    pub.stop();
    REQUIRE(!dev.isOpen());
}
