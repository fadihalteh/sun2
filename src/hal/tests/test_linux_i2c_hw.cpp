#include "actuators/PCA9685.hpp"

#include <cstdlib>
#include <iostream>
#include <optional>
#include <string>

namespace {

std::optional<std::string> getEnvString(const char* name) {
    const char* v = std::getenv(name);
    if (!v || std::string(v).empty()) {
        return std::nullopt;
    }
    return std::string(v);
}

std::optional<int> getEnvInt(const char* name) {
    auto s = getEnvString(name);
    if (!s) {
        return std::nullopt;
    }
    try {
        return std::stoi(*s);
    } catch (...) {
        return std::nullopt;
    }
}

std::optional<float> getEnvFloat(const char* name) {
    auto s = getEnvString(name);
    if (!s) {
        return std::nullopt;
    }
    try {
        return std::stof(*s);
    } catch (...) {
        return std::nullopt;
    }
}

} // namespace

int main() {
    // Require explicit opt-in.
    const auto run_hw = getEnvString("SOLAR_RUN_I2C_HW_TESTS");
    if (!run_hw || *run_hw != "1") {
        std::cerr << "[SKIP] SOLAR_RUN_I2C_HW_TESTS not enabled\n";
        return 77;
    }

    // Require explicit safe pulse.
    // Do not guess a mechanically safe value in code.
    const auto safe_pulse_us = getEnvFloat("SOLAR_HW_SAFE_PULSE_US");
    if (!safe_pulse_us) {
        std::cerr << "[SKIP] SOLAR_HW_SAFE_PULSE_US not set\n";
        return 77;
    }

    // Default to a single explicitly chosen safe channel.
    const int channel = getEnvInt("SOLAR_HW_SAFE_CHANNEL").value_or(0);

    if (channel < 0 || channel > 15) {
        std::cerr << "[FAIL] invalid SOLAR_HW_SAFE_CHANNEL: " << channel << "\n";
        return 1;
    }

    const auto i2c_dev = getEnvString("SOLAR_I2C_DEV").value_or("/dev/i2c-1");
    const int i2c_addr = getEnvInt("SOLAR_PCA9685_ADDR").value_or(0x40);

    solar::actuators::PCA9685::Config cfg{};
    cfg.device = i2c_dev;
    cfg.address = static_cast<std::uint8_t>(i2c_addr);
    cfg.frequency_hz = 50.0F;

    solar::actuators::PCA9685 pca{cfg};

    if (!pca.start()) {
        std::cerr << "[FAIL] PCA9685 start failed\n";
        return 1;
    }

    // Single safe write only.
    if (!pca.setPulseUs(channel, *safe_pulse_us)) {
        std::cerr << "[FAIL] PCA9685 safe write failed\n";
        pca.stop();
        return 1;
    }

    std::cout << "[PASS] Wrote safe pulse " << *safe_pulse_us
              << " us to channel " << channel << "\n";

    pca.stop();
    return 0;
}