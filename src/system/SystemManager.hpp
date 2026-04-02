#pragma once

/**
 * @file SystemManager.hpp
 * @brief Top-level runtime orchestrator for the tracker pipeline.
 *
 * SystemManager composes and coordinates the main event-driven stages:
 * - camera/frame input
 * - vision estimate generation
 * - controller setpoint generation
 * - kinematics command generation
 * - actuator output dispatch
 * - optional manual and IMU backends
 *
 * It is the runtime orchestration boundary. It is not a low-level device class
 * and it is not itself a GUI class.
 */

#include "actuators/ActuatorManager.hpp"
#include "actuators/ServoDriver.hpp"
#include "app/AppConfig.hpp"
#include "common/LatencyMonitor.hpp"
#include "common/Logger.hpp"
#include "common/ThreadSafeQueue.hpp"
#include "common/Types.hpp"
#include "control/Controller.hpp"
#include "control/Kinematics3RRS.hpp"
#include "control/ManualImuCoordinator.hpp"
#include "sensors/ICamera.hpp"
#include "system/TrackerState.hpp"
#include "vision/SunTracker.hpp"

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

namespace solar {

namespace control {
struct ImuSample;
}

/**
 * @brief Runtime orchestration class for the final tracker system.
 */
class SystemManager {
public:
    /**
     * @brief Observer callback for runtime state transitions.
     *
     * @param new_state Newly entered tracker state.
     */
    using StateObserver = std::function<void(TrackerState new_state)>;

    /**
     * @brief Observer callback for raw manual potentiometer samples.
     *
     * @param sample Latest completed manual sample.
     */
    using ManualObserver = std::function<void(const ManualPotSample& sample)>;

    /**
     * @brief Observer callback for IMU updates.
     *
     * @param sample Latest IMU sample.
     * @param tilt_rad Estimated platform tilt in radians.
     * @param valid True if the tilt estimate is valid.
     */
    using ImuObserver = std::function<void(const control::ImuSample& sample,
                                           float tilt_rad,
                                           bool valid)>;

    /**
     * @brief Observer callback for per-frame latency breakdown.
     *
     * @param frame_id Frame identifier.
     * @param cap_to_est_ms Capture-to-estimate latency in milliseconds.
     * @param est_to_ctrl_ms Estimate-to-control latency in milliseconds.
     * @param ctrl_to_act_ms Control-to-actuation latency in milliseconds.
     */
    using LatencyObserver = std::function<void(std::uint64_t frame_id,
                                               double cap_to_est_ms,
                                               double est_to_ctrl_ms,
                                               double ctrl_to_act_ms)>;

    /**
     * @brief Alias for the manual command ownership policy used by the
     *        manual/IMU coordinator.
     */
    using ManualCommandSource = ManualImuCoordinator::ManualCommandSource;

    /**
     * @brief Construct a runtime using the top-level application configuration.
     *
     * @param log Shared logger.
     * @param camera Camera backend owned by the runtime.
     * @param cfg Full runtime configuration.
     */
    SystemManager(Logger& log,
                  std::unique_ptr<ICamera> camera,
                  const app::AppConfig& cfg);

    /**
     * @brief Construct a runtime from explicit stage configurations.
     *
     * This overload mainly supports tests or targeted composition without a
     * full @ref app::AppConfig object.
     *
     * @param log Shared logger.
     * @param camera Camera backend owned by the runtime.
     * @param trackerCfg Vision-stage configuration.
     * @param controllerCfg Controller-stage configuration.
     * @param kinCfg Kinematics-stage configuration.
     * @param actCfg Actuator-manager configuration.
     * @param drvCfg Servo-driver configuration.
     */
    SystemManager(Logger& log,
                  std::unique_ptr<ICamera> camera,
                  SunTracker::Config trackerCfg,
                  Controller::Config controllerCfg,
                  Kinematics3RRS::Config kinCfg,
                  ActuatorManager::Config actCfg,
                  ServoDriver::Config drvCfg);

    /**
     * @brief Destructor stops the runtime if still running.
     */
    ~SystemManager();

    SystemManager(const SystemManager&) = delete;
    SystemManager& operator=(const SystemManager&) = delete;

    /**
     * @brief Start the runtime threads and optional backends.
     *
     * @return True on successful startup.
     */
    bool start();

    /**
     * @brief Stop the runtime and all owned worker threads/backends.
     */
    void stop();

    /**
     * @brief Return the current tracker state.
     *
     * @return Current state snapshot.
     */
    TrackerState state() const;

    /**
     * @brief Request entry into manual mode.
     */
    void enterManual();

    /**
     * @brief Request exit from manual mode back to automatic processing.
     */
    void exitManual();

    /**
     * @brief Submit an explicit manual setpoint.
     *
     * @param tilt_rad Requested tilt in radians.
     * @param pan_rad Requested pan in radians.
     */
    void setManualSetpoint(float tilt_rad, float pan_rad);

    /**
     * @brief Select which manual command source currently owns manual control.
     *
     * @param src Manual command source.
     */
    void setManualCommandSource(ManualCommandSource src);

    /**
     * @brief Update the runtime tracker threshold.
     *
     * @param threshold New threshold value.
     */
    void setTrackerThreshold(std::uint8_t threshold);

    /**
     * @brief Update the minimum confidence accepted by the controller.
     *
     * @param confidence New minimum confidence in the range [0, 1].
     */
    void setMinConfidence(float confidence);

    /**
     * @brief Register an observer for raw frames.
     *
     * @param cb Frame observer callback.
     */
    void registerFrameObserver(ICamera::FrameCallback cb);

    /**
     * @brief Register an observer for vision estimates.
     *
     * @param cb Estimate observer callback.
     */
    void registerEstimateObserver(SunTracker::EstimateCallback cb);

    /**
     * @brief Register an observer for controller setpoints.
     *
     * @param cb Setpoint observer callback.
     */
    void registerSetpointObserver(Controller::SetpointCallback cb);

    /**
     * @brief Register an observer for kinematics/actuator commands.
     *
     * @param cb Command observer callback.
     */
    void registerCommandObserver(Kinematics3RRS::CommandCallback cb);

    /**
     * @brief Register a runtime-state observer.
     *
     * @param cb State observer callback.
     */
    void registerStateObserver(StateObserver cb);

    /**
     * @brief Register an observer for manual potentiometer samples.
     *
     * @param cb Manual sample observer callback.
     */
    void registerManualObserver(ManualObserver cb);

    /**
     * @brief Register an observer for IMU updates.
     *
     * @param cb IMU observer callback.
     */
    void registerImuObserver(ImuObserver cb);

    /**
     * @brief Register an observer for per-frame latency breakdown.
     *
     * @param cb Latency observer callback.
     */
    void registerLatencyObserver(LatencyObserver cb);

private:
    /**
     * @brief Internal helper owning optional hardware backends.
     *
     * This keeps backend bring-up/teardown out of the main orchestration class
     * so SystemManager can focus on pipeline and state coordination.
     */
    class BackendCoordinator;

    void initialiseCallbacks_();
    void controlLoop_();
    void actuatorLoop_();

    void onFrame_(const FrameEvent& fe);
    void onManualPotSample_(const ManualPotSample& sample);
    void onImuSample_(const control::ImuSample& sample);

    bool canAutoProcess_(TrackerState s) const noexcept;
    void setState_(TrackerState s);

    std::uint64_t nextSyntheticFrameId_() noexcept;

    void applyParkOnce_(float deg);
    void applyNeutralOnce_();

private:
    Logger& log_;
    std::unique_ptr<ICamera> camera_;

    SunTracker tracker_;
    Controller controller_;
    Kinematics3RRS kinematics_;
    ActuatorManager actuatorMgr_;
    float startup_park_deg_{41.0F};
    ServoDriver driver_;
    LatencyMonitor latency_;
    app::AppConfig config_;

    ThreadSafeQueue<FrameEvent> frame_q_{2};
    ThreadSafeQueue<ActuatorCommand> cmd_q_{8};

    std::thread control_thread_;
    std::thread actuator_thread_;

    std::atomic<bool> running_{false};
    std::atomic<TrackerState> state_{TrackerState::IDLE};
    std::atomic<std::uint64_t> next_synthetic_frame_id_{1};

    float min_confidence_{0.4F};

    ManualImuCoordinator manual_imu_;
    std::unique_ptr<BackendCoordinator> backends_;

    mutable std::mutex kin_mtx_;
    mutable std::mutex obs_mtx_;

    ICamera::FrameCallback frame_obs_{};
    SunTracker::EstimateCallback estimate_obs_{};
    Controller::SetpointCallback setpoint_obs_{};
    Kinematics3RRS::CommandCallback command_obs_{};
    StateObserver state_obs_{};
    ManualObserver manual_obs_{};
    ImuObserver imu_obs_{};
    LatencyObserver latency_obs_{};
};

} // namespace solar