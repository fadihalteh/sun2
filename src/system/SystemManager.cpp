#include "system/SystemManager.hpp"

#include "control/ImuTiltEstimator.hpp"
#include "hal/LinuxI2CDevice.hpp"
#include "sensors/imu/IIMU.hpp"
#include "sensors/imu/Mpu6050Publisher.hpp"
#include "sensors/manual/ADS1115ManualInput.hpp"

#include <chrono>
#include <string>
#include <utility>

namespace solar {

namespace {

std::string stateToMsg(const TrackerState s) {
    return std::string("STATE -> ") + toString(s);
}

app::AppConfig makeNoBackendConfig_(const Controller::Config& controllerCfg) {
    app::AppConfig cfg = app::defaultConfig();
    cfg.manual_input_backend = app::ManualInputBackend::None;
    cfg.ads1115_manual.enabled = false;
    cfg.imu_backend = app::ImuBackend::None;
    cfg.mpu6050.enabled = false;
    cfg.imu_feedback_mode = app::ImuFeedbackMode::Disabled;
    cfg.controller = controllerCfg;
    return cfg;
}

} // namespace

class SystemManager::BackendCoordinator {
public:
    using ManualCallback = std::function<void(const ManualPotSample&)>;
    using ImuCallback = std::function<void(const control::ImuSample&)>;

    struct StartResult {
        bool manual_ok{true};
        bool imu_ok{true};
    };

    BackendCoordinator(const app::AppConfig& cfg,
                       ManualCallback manual_cb,
                       ImuCallback imu_cb)
        : config_(cfg),
          manual_callback_(std::move(manual_cb)),
          imu_callback_(std::move(imu_cb)),
          manual_backend_requested_(cfg.manual_input_backend == app::ManualInputBackend::ADS1115
                                    && cfg.ads1115_manual.enabled),
          imu_backend_requested_(cfg.imu_backend == app::ImuBackend::Mpu6050Gpio
                                 && cfg.mpu6050.enabled) {
    }

    StartResult start(Logger& log) {
        StartResult result{};

        if (!startManual_(log)) {
            result.manual_ok = false;
            return result;
        }

        if (!startImu_(log)) {
            result.imu_ok = false;
        }

        return result;
    }

    void stop() {
        stopImu_();
        stopManual_();
    }

private:
    class ImuCallbackForwarder final : public IIMU::CallbackInterface {
    public:
        explicit ImuCallbackForwarder(ImuCallback callback)
            : callback_(std::move(callback)) {
        }

        void hasSample(const IImuSample& sample) override {
            control::ImuSample mapped{};
            mapped.ax = sample.ax_mps2;
            mapped.ay = sample.ay_mps2;
            mapped.az = sample.az_mps2;
            mapped.timestamp_us = sample.timestamp_us;
            mapped.valid = sample.valid;

            if (callback_) {
                callback_(mapped);
            }
        }

    private:
        ImuCallback callback_{};
    };

    bool startManual_(Logger& log) {
        if (!manual_backend_requested_) {
            return true;
        }

        ADS1115ManualInputSettings s{};
        s.enabled = config_.ads1115_manual.enabled;
        s.i2c_bus = config_.ads1115_manual.i2c_bus;
        s.i2c_address = config_.ads1115_manual.i2c_address;
        s.gpio_chip_index = config_.ads1115_manual.gpio_chip_index;
        s.alert_rdy_gpio = config_.ads1115_manual.alert_rdy_gpio;
        s.tilt_channel = config_.ads1115_manual.tilt_channel;
        s.pan_channel = config_.ads1115_manual.pan_channel;
        s.spare_channel = config_.ads1115_manual.spare_channel;
        s.full_scale_voltage = config_.ads1115_manual.full_scale_voltage;
        s.sample_rate_hz = config_.ads1115_manual.sample_rate_hz;

        manual_input_ = std::make_unique<ADS1115ManualInput>(s);
        manual_input_->registerCallback([this](const ManualPotSample& sample) {
            if (manual_callback_) {
                manual_callback_(sample);
            }
        });

        if (!manual_input_->start()) {
            manual_input_.reset();
            return false;
        }

        log.info("SystemManager: manual ADS1115 backend started");
        return true;
    }

    void stopManual_() {
        if (manual_input_) {
            manual_input_->stop();
            manual_input_.reset();
        }
    }

    bool startImu_(Logger& log) {
        if (!imu_backend_requested_) {
            return true;
        }

        imu_i2c_ = std::make_unique<LinuxI2CDevice>();

        Mpu6050Config cfg{};
        cfg.i2c.bus = config_.mpu6050.i2c_bus;
        cfg.i2c.address = config_.mpu6050.i2c_address;
        cfg.gpio_pin_no = static_cast<int>(config_.mpu6050.data_ready_gpio);
        cfg.gpio_chip_no = config_.mpu6050.gpio_chip_index;
        cfg.who_am_i_reg = config_.mpu6050.who_am_i_reg;
        cfg.who_am_i_expected = config_.mpu6050.who_am_i_expected;
        cfg.power_mgmt_reg = config_.mpu6050.power_mgmt_reg;
        cfg.sample_rate_div_reg = config_.mpu6050.sample_rate_div_reg;
        cfg.config_reg = config_.mpu6050.config_reg;
        cfg.gyro_config_reg = config_.mpu6050.gyro_config_reg;
        cfg.accel_config_reg = config_.mpu6050.accel_config_reg;
        cfg.int_enable_reg = config_.mpu6050.int_enable_reg;
        cfg.int_status_reg = config_.mpu6050.int_status_reg;
        cfg.sample_start_reg = config_.mpu6050.sample_start_reg;
        cfg.wake_value = config_.mpu6050.wake_value;
        cfg.sample_rate_div_value = config_.mpu6050.sample_rate_div_value;
        cfg.config_value = config_.mpu6050.config_value;
        cfg.gyro_config_value = config_.mpu6050.gyro_config_value;
        cfg.accel_config_value = config_.mpu6050.accel_config_value;
        cfg.int_enable_value = config_.mpu6050.int_enable_value;
        cfg.data_ready_mask = config_.mpu6050.data_ready_mask;
        cfg.accel_lsb_per_g = config_.mpu6050.accel_lsb_per_g;
        cfg.gyro_lsb_per_dps = config_.mpu6050.gyro_lsb_per_dps;
        cfg.startup_discard_samples = config_.mpu6050.startup_discard_samples;

        imu_input_ = std::make_unique<Mpu6050Publisher>(*imu_i2c_, cfg);
        imu_callback_forwarder_ = std::make_unique<ImuCallbackForwarder>(imu_callback_);
        imu_input_->registerEventCallback(imu_callback_forwarder_.get());

        if (!imu_input_->start()) {
            imu_input_.reset();
            imu_callback_forwarder_.reset();
            imu_i2c_.reset();
            return false;
        }

        log.info("SystemManager: MPU6050 backend started");
        return true;
    }

    void stopImu_() {
        if (imu_input_) {
            imu_input_->stop();
            imu_input_.reset();
        }
        imu_callback_forwarder_.reset();
        imu_i2c_.reset();
    }

private:
    app::AppConfig config_{};
    ManualCallback manual_callback_{};
    ImuCallback imu_callback_{};

    bool manual_backend_requested_{false};
    bool imu_backend_requested_{false};

    std::unique_ptr<ADS1115ManualInput> manual_input_{};
    std::unique_ptr<LinuxI2CDevice> imu_i2c_{};
    std::unique_ptr<IIMU> imu_input_{};
    std::unique_ptr<ImuCallbackForwarder> imu_callback_forwarder_{};
};

SystemManager::SystemManager(
    Logger& log,
    std::unique_ptr<ICamera> camera,
    const app::AppConfig& cfg)
    : log_(log),
      camera_(std::move(camera)),
      tracker_(log_, cfg.tracker),
      controller_(log_, cfg.controller),
      kinematics_(log_, cfg.kinematics),
      actuatorMgr_(log_, cfg.actuator),
      startup_park_deg_(cfg.servo.ch[0].neutral_deg),
      driver_(log_, cfg.servo),
      latency_(log_),
      config_(cfg),
      min_confidence_(cfg.controller.min_confidence),
      manual_imu_(cfg.manual_mapping,
                  cfg.imu_feedback,
                  cfg.imu_feedback_mode,
                  cfg.controller.max_tilt_rad,
                  cfg.controller.max_pan_rad),
      backends_(std::make_unique<BackendCoordinator>(
          cfg,
          [this](const ManualPotSample& sample) { onManualPotSample_(sample); },
          [this](const control::ImuSample& sample) { onImuSample_(sample); })) {
    initialiseCallbacks_();
}

SystemManager::SystemManager(
    Logger& log,
    std::unique_ptr<ICamera> camera,
    SunTracker::Config trackerCfg,
    Controller::Config controllerCfg,
    Kinematics3RRS::Config kinCfg,
    ActuatorManager::Config actCfg,
    ServoDriver::Config drvCfg)
    : log_(log),
      camera_(std::move(camera)),
      tracker_(log_, trackerCfg),
      controller_(log_, controllerCfg),
      kinematics_(log_, kinCfg),
      actuatorMgr_(log_, actCfg),
      startup_park_deg_(drvCfg.ch[0].neutral_deg),
      driver_(log_, drvCfg),
      latency_(log_),
      config_(makeNoBackendConfig_(controllerCfg)),
      min_confidence_(controllerCfg.min_confidence),
      manual_imu_({3.3F, false, false, 0.0F, 0.0F, 20.0F, 20.0F},
                  {0.12F, 0.01745329252F, 0.01745329252F},
                  app::ImuFeedbackMode::Disabled,
                  controllerCfg.max_tilt_rad,
                  controllerCfg.max_pan_rad),
      backends_(std::make_unique<BackendCoordinator>(
          config_,
          [this](const ManualPotSample& sample) { onManualPotSample_(sample); },
          [this](const control::ImuSample& sample) { onImuSample_(sample); })) {
    initialiseCallbacks_();
}

SystemManager::~SystemManager() {
    stop();
}

void SystemManager::setManualCommandSource(const ManualCommandSource src) {
    manual_imu_.setManualCommandSource(src);
}

void SystemManager::initialiseCallbacks_() {
    latency_.registerObserver([this](std::uint64_t frame_id,
                                     double cap_to_est_ms,
                                     double est_to_ctrl_ms,
                                     double ctrl_to_act_ms) {
        LatencyObserver cb;
        {
            std::lock_guard<std::mutex> lk(obs_mtx_);
            cb = latency_obs_;
        }
        if (cb) {
            cb(frame_id, cap_to_est_ms, est_to_ctrl_ms, ctrl_to_act_ms);
        }
    });

    if (camera_) {
        camera_->registerFrameCallback([this](const FrameEvent& fe) {
            onFrame_(fe);
        });
    }

    tracker_.registerEstimateCallback([this](const SunEstimate& est) {
        latency_.onEstimate(est.frame_id, est.t_estimate);

        SunTracker::EstimateCallback cb;
        {
            std::lock_guard<std::mutex> lk(obs_mtx_);
            cb = estimate_obs_;
        }
        if (cb) {
            cb(est);
        }

        if (!running_.load()) {
            return;
        }

        const TrackerState current = state_.load();
        if (!canAutoProcess_(current)) {
            return;
        }

        setState_(est.confidence >= min_confidence_
                      ? TrackerState::TRACKING
                      : TrackerState::SEARCHING);

        if (!canAutoProcess_(state_.load())) {
            return;
        }

        controller_.onEstimate(est);
    });

    controller_.registerSetpointCallback([this](const PlatformSetpoint& sp) {
        latency_.onControl(sp.frame_id, sp.t_control);

        Controller::SetpointCallback cb;
        {
            std::lock_guard<std::mutex> lk(obs_mtx_);
            cb = setpoint_obs_;
        }
        if (cb) {
            cb(sp);
        }

        if (!running_.load()) {
            return;
        }

        if (!canAutoProcess_(state_.load())) {
            return;
        }

        const PlatformSetpoint corrected = manual_imu_.applyImuCorrection(sp);

        {
            std::lock_guard<std::mutex> lk(kin_mtx_);
            kinematics_.onSetpoint(corrected);
        }
    });

    kinematics_.registerCommandCallback([this](const ActuatorCommand& cmd) {
        Kinematics3RRS::CommandCallback cb;
        {
            std::lock_guard<std::mutex> lk(obs_mtx_);
            cb = command_obs_;
        }
        if (cb) {
            cb(cmd);
        }

        if (cmd.status == CommandStatus::KinematicsInvalidConfig) {
            log_.error("SystemManager: kinematics invalid; entering FAULT");
            setState_(TrackerState::FAULT);
            return;
        }

        (void)cmd_q_.push_latest(cmd);
    });

    actuatorMgr_.registerSafeCommandCallback([this](const ActuatorCommand& safeCmd) {
        auto out = safeCmd;
        out.t_actuate = std::chrono::steady_clock::now();

        latency_.onActuate(out.frame_id, out.t_actuate);
        driver_.apply(out);
    });
}

bool SystemManager::start() {
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true)) {
        return true;
    }

    setState_(TrackerState::STARTUP);

    if (!camera_) {
        log_.error("SystemManager: camera is null");
        setState_(TrackerState::FAULT);
        running_.store(false);
        return false;
    }

    if (!driver_.start()) {
        log_.error("SystemManager: ServoDriver start failed");
        setState_(TrackerState::FAULT);
        running_.store(false);
        return false;
    }

    frame_q_.clear();
    frame_q_.reset();
    cmd_q_.clear();
    cmd_q_.reset();
    actuatorMgr_.resetHistory();
    manual_imu_.reset();

    control_thread_ = std::thread([this] {
        controlLoop_();
    });

    actuator_thread_ = std::thread([this] {
        actuatorLoop_();
    });

    const BackendCoordinator::StartResult backend_result = backends_->start(log_);
    if (!backend_result.manual_ok) {
        log_.error("SystemManager: manual backend start failed");
        stop();
        setState_(TrackerState::FAULT);
        return false;
    }

    if (!backend_result.imu_ok) {
        if (manual_imu_.imuFeedbackMode() == app::ImuFeedbackMode::Live) {
            log_.error("SystemManager: IMU backend start failed in LIVE mode");
            stop();
            setState_(TrackerState::FAULT);
            return false;
        }

        log_.warn("SystemManager: IMU backend start failed; continuing without IMU");
        manual_imu_.setImuFeedbackMode(app::ImuFeedbackMode::Disabled);
    }

    if (!camera_->start()) {
        log_.error("SystemManager: camera start failed");
        stop();
        setState_(TrackerState::FAULT);
        return false;
    }

    setState_(TrackerState::NEUTRAL);
    applyParkOnce_(startup_park_deg_);

    if (config_.startup_mode == app::StartupMode::Manual) {
        setState_(TrackerState::MANUAL);
        manual_imu_.setManualCommandSource(ManualCommandSource::Pot);
    } else {
        setState_(TrackerState::SEARCHING);
    }

    log_.info("SystemManager started");
    return true;
}

void SystemManager::stop() {
    const bool was_running = running_.exchange(false);
    if (!was_running) {
        return;
    }

    setState_(TrackerState::STOPPING);

    if (camera_) {
        camera_->stop();
    }

    backends_->stop();

    frame_q_.stop();
    cmd_q_.stop();

    if (control_thread_.joinable()) {
        control_thread_.join();
    }
    if (actuator_thread_.joinable()) {
        actuator_thread_.join();
    }

    applyNeutralOnce_();
    driver_.stop();
    latency_.printSummary();

    setState_(TrackerState::IDLE);
    log_.info("SystemManager stopped");
}

void SystemManager::controlLoop_() {
    while (true) {
        const auto item = frame_q_.wait_pop();
        if (!item.has_value()) {
            break;
        }

        if (!running_.load()) {
            break;
        }

        if (!canAutoProcess_(state_.load())) {
            continue;
        }

        tracker_.onFrame(*item);
    }
}

void SystemManager::actuatorLoop_() {
    while (true) {
        const auto item = cmd_q_.wait_pop();
        if (!item.has_value()) {
            break;
        }

        if (!running_.load()) {
            break;
        }

        actuatorMgr_.onCommand(*item);
    }
}

void SystemManager::onFrame_(const FrameEvent& fe) {
    latency_.onCapture(fe.frame_id, fe.t_capture);

    ICamera::FrameCallback cb;
    {
        std::lock_guard<std::mutex> lk(obs_mtx_);
        cb = frame_obs_;
    }
    if (cb) {
        cb(fe);
    }

    (void)frame_q_.push_latest(fe);
}

void SystemManager::onManualPotSample_(const ManualPotSample& sample) {
    ManualObserver manual_cb;
    {
        std::lock_guard<std::mutex> lk(obs_mtx_);
        manual_cb = manual_obs_;
    }
    if (manual_cb) {
        manual_cb(sample);
    }

    if (!running_.load()) {
        return;
    }

    PlatformSetpoint sp{};
    if (!manual_imu_.buildManualSetpointFromPot(sample,
                                                state_.load(),
                                                nextSyntheticFrameId_(),
                                                std::chrono::steady_clock::now(),
                                                sp)) {
        return;
    }

    Controller::SetpointCallback cb;
    {
        std::lock_guard<std::mutex> lk(obs_mtx_);
        cb = setpoint_obs_;
    }
    if (cb) {
        cb(sp);
    }

    latency_.onControl(sp.frame_id, sp.t_control);

    {
        std::lock_guard<std::mutex> lk(kin_mtx_);
        kinematics_.onSetpoint(sp);
    }
}

void SystemManager::onImuSample_(const control::ImuSample& sample) {
    if (!running_.load()) {
        return;
    }

    manual_imu_.updateImuSample(sample);

    float tilt = 0.0F;
    const bool valid = manual_imu_.latestImuTiltRad(tilt);

    ImuObserver imu_cb;
    {
        std::lock_guard<std::mutex> lk(obs_mtx_);
        imu_cb = imu_obs_;
    }
    if (imu_cb) {
        imu_cb(sample, tilt, valid);
    }
}

TrackerState SystemManager::state() const {
    return state_.load();
}

void SystemManager::enterManual() {
    manual_imu_.setManualCommandSource(ManualCommandSource::Pot);
    setState_(TrackerState::MANUAL);
}

void SystemManager::exitManual() {
    if (!running_.load()) {
        setState_(TrackerState::IDLE);
        return;
    }
    setState_(TrackerState::SEARCHING);
}

void SystemManager::setManualSetpoint(const float tilt_rad, const float pan_rad) {
    if (!running_.load()) {
        return;
    }

    PlatformSetpoint sp{};
    if (!manual_imu_.buildManualSetpointFromGui(tilt_rad,
                                                pan_rad,
                                                state_.load(),
                                                nextSyntheticFrameId_(),
                                                std::chrono::steady_clock::now(),
                                                sp)) {
        return;
    }

    Controller::SetpointCallback cb;
    {
        std::lock_guard<std::mutex> lk(obs_mtx_);
        cb = setpoint_obs_;
    }
    if (cb) {
        cb(sp);
    }

    latency_.onControl(sp.frame_id, sp.t_control);

    {
        std::lock_guard<std::mutex> lk(kin_mtx_);
        kinematics_.onSetpoint(sp);
    }
}

void SystemManager::setTrackerThreshold(const std::uint8_t thr) {
    tracker_.setThreshold(thr);
}

void SystemManager::setMinConfidence(const float confidence) {
    min_confidence_ = confidence;
}

void SystemManager::registerFrameObserver(ICamera::FrameCallback cb) {
    std::lock_guard<std::mutex> lk(obs_mtx_);
    frame_obs_ = std::move(cb);
}

void SystemManager::registerEstimateObserver(SunTracker::EstimateCallback cb) {
    std::lock_guard<std::mutex> lk(obs_mtx_);
    estimate_obs_ = std::move(cb);
}

void SystemManager::registerSetpointObserver(Controller::SetpointCallback cb) {
    std::lock_guard<std::mutex> lk(obs_mtx_);
    setpoint_obs_ = std::move(cb);
}

void SystemManager::registerCommandObserver(Kinematics3RRS::CommandCallback cb) {
    std::lock_guard<std::mutex> lk(obs_mtx_);
    command_obs_ = std::move(cb);
}

void SystemManager::registerStateObserver(StateObserver cb) {
    std::lock_guard<std::mutex> lk(obs_mtx_);
    state_obs_ = std::move(cb);
}

void SystemManager::registerManualObserver(ManualObserver cb) {
    std::lock_guard<std::mutex> lk(obs_mtx_);
    manual_obs_ = std::move(cb);
}

void SystemManager::registerImuObserver(ImuObserver cb) {
    std::lock_guard<std::mutex> lk(obs_mtx_);
    imu_obs_ = std::move(cb);
}

void SystemManager::registerLatencyObserver(LatencyObserver cb) {
    std::lock_guard<std::mutex> lk(obs_mtx_);
    latency_obs_ = std::move(cb);
}

void SystemManager::setState_(const TrackerState s) {
    const TrackerState prev = state_.exchange(s);
    if (prev != s) {
        log_.info(stateToMsg(s));

        StateObserver cb;
        {
            std::lock_guard<std::mutex> lk(obs_mtx_);
            cb = state_obs_;
        }
        if (cb) {
            cb(s);
        }
    }
}

bool SystemManager::canAutoProcess_(const TrackerState s) const noexcept {
    return s == TrackerState::SEARCHING || s == TrackerState::TRACKING;
}

std::uint64_t SystemManager::nextSyntheticFrameId_() noexcept {
    return next_synthetic_frame_id_.fetch_add(1U, std::memory_order_relaxed);
}

void SystemManager::applyNeutralOnce_() {
    PlatformSetpoint sp{};
    sp.frame_id = nextSyntheticFrameId_();
    sp.t_control = std::chrono::steady_clock::now();
    sp.tilt_rad = 0.0F;
    sp.pan_rad = 0.0F;

    {
        std::lock_guard<std::mutex> lk(kin_mtx_);
        kinematics_.onSetpoint(sp);
    }

    auto maybe_cmd = cmd_q_.try_pop();
    if (maybe_cmd.has_value()) {
        actuatorMgr_.onCommand(*maybe_cmd);
    }
}

void SystemManager::applyParkOnce_(const float servo_deg) {
    ActuatorCommand cmd{};
    cmd.frame_id = nextSyntheticFrameId_();
    cmd.t_actuate = std::chrono::steady_clock::now();
    cmd.status = CommandStatus::Ok;
    cmd.actuator_targets = {servo_deg, servo_deg, servo_deg};

    actuatorMgr_.onCommand(cmd);
}

} // namespace solar