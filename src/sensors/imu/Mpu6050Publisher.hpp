#pragma once

/**
 * @file Mpu6050Publisher.hpp
 * @brief MPU6050 publisher using GPIO data-ready events and Linux I2C.
 *
 * This class is hardware-facing only.
 *
 * Responsibilities:
 * - initialise MPU6050 registers
 * - react to GPIO data-ready events
 * - read one IMU sample from the device
 * - publish that sample via callback
 *
 * Non-responsibilities:
 * - estimation
 * - control
 * - state-machine policy
 */

#include "external/libgpiod_event_demo/gpioevent.h"
#include "hal/II2CDevice.hpp"
#include "sensors/imu/IIMU.hpp"

#include <cstddef>
#include <cstdint>

class GPIOPin;

namespace solar {

/**
 * @brief Runtime configuration for the MPU6050 publisher.
 */
struct Mpu6050Config {
    I2CDeviceSettings i2c{};                    ///< Linux I2C bus/address.
    int gpio_pin_no{0};                        ///< Data-ready GPIO line.
    int gpio_chip_no{0};                       ///< GPIO chip index.

    std::uint8_t who_am_i_reg{0x75U};          ///< WHO_AM_I register.
    std::uint8_t who_am_i_expected{0x70U};     ///< Expected device id.

    std::uint8_t power_mgmt_reg{0x6BU};        ///< Power-management register.
    std::uint8_t sample_rate_div_reg{0x19U};   ///< Sample-rate divider register.
    std::uint8_t config_reg{0x1AU};            ///< DLPF config register.
    std::uint8_t gyro_config_reg{0x1BU};       ///< Gyro config register.
    std::uint8_t accel_config_reg{0x1CU};      ///< Accel config register.
    std::uint8_t int_enable_reg{0x38U};        ///< Interrupt-enable register.
    std::uint8_t int_status_reg{0x3AU};        ///< Interrupt-status register.
    std::uint8_t sample_start_reg{0x3BU};      ///< Burst-read start register.

    std::uint8_t wake_value{0x00U};            ///< Wake-up register value.
    std::uint8_t sample_rate_div_value{0x07U}; ///< Sample-rate divider value.
    std::uint8_t config_value{0x03U};          ///< DLPF config value.
    std::uint8_t gyro_config_value{0x00U};     ///< Gyro full-scale value.
    std::uint8_t accel_config_value{0x00U};    ///< Accel full-scale value.
    std::uint8_t int_enable_value{0x01U};      ///< Data-ready interrupt enable.
    std::uint8_t data_ready_mask{0x01U};       ///< Data-ready bit mask.

    float accel_lsb_per_g{16384.0F};           ///< Accel sensitivity.
    float gyro_lsb_per_dps{131.0F};            ///< Gyro sensitivity.

    std::size_t startup_discard_samples{8U};   ///< Samples discarded after startup.
};

/**
 * @brief Event-driven MPU6050 publisher.
 */
class Mpu6050Publisher final : public IIMU {
public:
    /**
     * @brief Construct the publisher.
     *
     * @param device Low-level I2C device.
     * @param config MPU6050 runtime configuration.
     */
    Mpu6050Publisher(II2CDevice& device, Mpu6050Config config = {});

    /**
     * @brief Destructor stops acquisition if still running.
     */
    ~Mpu6050Publisher() override;

    Mpu6050Publisher(const Mpu6050Publisher&) = delete;
    Mpu6050Publisher& operator=(const Mpu6050Publisher&) = delete;

    /**
     * @brief Register the callback sink receiving IMU samples.
     *
     * @param cb Callback sink.
     */
    void registerEventCallback(CallbackInterface* cb) override;

    /**
     * @brief Start acquisition.
     *
     * @return True on success.
     */
    bool start() override;

    /**
     * @brief Stop acquisition.
     */
    void stop() override;

private:
    bool initialise_();
    bool readSample_(IImuSample& sample);
    void onGpioEvent_(const gpiod::edge_event& event);

    static std::int16_t toInt16_(std::uint8_t msb, std::uint8_t lsb);
    static std::uint64_t monotonicNowUs_();

private:
    II2CDevice& device_;
    Mpu6050Config config_{};
    GPIOPin gpio_pin_{};
    CallbackInterface* callback_{nullptr};

    bool running_{false};
    std::size_t discard_remaining_{0U};
};

} // namespace solar
