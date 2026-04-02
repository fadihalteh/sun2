#pragma once

/**
 * @file Kinematics3RRS.hpp
 * @brief 3-RRS platform kinematics stage converting setpoints into servo commands.
 *
 * This class sits between the controller and the actuator stage. It receives a
 * platform setpoint, computes servo targets, and emits an @ref ActuatorCommand
 * via callback.
 */

#include "common/Logger.hpp"
#include "common/Types.hpp"

#include <array>
#include <cstdint>
#include <functional>
#include <mutex>

namespace solar {

/**
 * @brief Kinematics stage for the 3-RRS platform.
 */
class Kinematics3RRS {
public:
    /**
     * @brief Callback receiving one actuator command.
     *
     * @param cmd Computed actuator command.
     */
    using CommandCallback = std::function<void(const ActuatorCommand& cmd)>;

    /**
     * @brief Geometry and calibration parameters for the 3-RRS mechanism.
     */
    struct Config {
        float base_radius_m{0.20f};     ///< Base platform radius in metres.
        float platform_radius_m{0.12f}; ///< Moving platform radius in metres.
        float home_height_m{0.18f};     ///< Neutral/home platform height in metres.
        float horn_length_m{0.10f};     ///< Servo horn length in metres.
        float rod_length_m{0.18f};      ///< Connecting rod length in metres.

        std::array<float, 3> base_theta_deg{0.f, 120.f, 240.f}; ///< Base anchor angles in degrees.
        std::array<float, 3> plat_theta_deg{0.f, 120.f, 240.f}; ///< Platform anchor angles in degrees.

        std::array<float, 3> servo_neutral_deg{90.f, 90.f, 90.f}; ///< Servo neutral calibration angles.
        std::array<int, 3> servo_dir{-1, -1, -1};                 ///< Servo sign convention per axis.
    };

    /**
     * @brief Construct the kinematics stage.
     *
     * @param log Shared logger.
     * @param cfg Kinematics configuration.
     */
    Kinematics3RRS(Logger& log, Config cfg);

    Kinematics3RRS(const Kinematics3RRS&) = delete;
    Kinematics3RRS& operator=(const Kinematics3RRS&) = delete;

    /**
     * @brief Register the output command callback.
     *
     * @param cb Callback receiving computed actuator commands.
     */
    void registerCommandCallback(CommandCallback cb);

    /**
     * @brief Return the current configuration.
     *
     * @return Current kinematics configuration.
     */
    Config config() const;

    /**
     * @brief Process one platform setpoint.
     *
     * @param sp Desired platform setpoint.
     */
    void onSetpoint(const PlatformSetpoint& sp);

private:
    void computeIK_(const PlatformSetpoint& sp);
    void emitCommand_(const ActuatorCommand& cmd);

    std::array<float, 3> q_prev_{0.f, 0.f, 0.f};
    std::array<float, 3> last_valid_deg_{90.f, 90.f, 90.f};

    Logger& log_;
    Config cfg_;

    mutable std::mutex cbMtx_;
    CommandCallback cmdCb_{};
};

} // namespace solar
