#include "app/AppConfig.hpp"

namespace {
constexpr float kRigParkDeg = 41.0F;
constexpr float kOneDegRad = 0.01745329252F;
}

namespace solar::app {

AppConfig defaultConfig() {
    AppConfig cfg{};

#if SOLAR_HAVE_LIBCAMERA
    cfg.camera_backend = CameraBackend::Libcamera;
    cfg.servo.startup_policy = ServoDriver::StartupPolicy::RequireHardware;
#else
    cfg.camera_backend = CameraBackend::Simulated;
    cfg.servo.startup_policy = ServoDriver::StartupPolicy::LogOnly;
#endif

    cfg.startup_mode = StartupMode::Auto;

    // Hardware manual input is part of the baseline runtime on the target rig.
    cfg.manual_input_backend = ManualInputBackend::ADS1115;
    cfg.ads1115_manual.enabled = true;

    // The IMU backend is enabled by default. Headless mode starts in SHADOW so
    // the system can continue operating if IMU feedback is unavailable.
    cfg.imu_backend = ImuBackend::Mpu6050Gpio;
    cfg.mpu6050.enabled = true;
    cfg.imu_feedback_mode = ImuFeedbackMode::Shadow;
    cfg.imu_feedback.gain = 0.12F;
    cfg.imu_feedback.deadband_rad = kOneDegRad;        // 1 degree
    cfg.imu_feedback.max_correction_rad = kOneDegRad;  // 1 degree

    // Vision defaults.
    cfg.tracker.threshold = 200U;
    cfg.tracker.min_pixels = 10U;
    cfg.tracker.confidence_scale = 1.0F;

    // Keep controller dimensions aligned with the selected camera defaults.
    cfg.controller.image_width = 640;
    cfg.controller.image_height = 480;
    cfg.controller.k_pan = 0.8F;
    cfg.controller.k_tilt = 0.8F;
    cfg.controller.max_pan_rad = 0.35F;
    cfg.controller.max_tilt_rad = 0.35F;
    cfg.controller.deadband = 0.0F;
    cfg.controller.min_confidence = 0.4F;

    // Stewart-platform geometry and calibration.
    cfg.kinematics.base_radius_m = 0.20F;
    cfg.kinematics.platform_radius_m = 0.12F;
    cfg.kinematics.home_height_m = 0.18F;
    cfg.kinematics.horn_length_m = 0.10F;
    cfg.kinematics.rod_length_m = 0.18F;
    cfg.kinematics.base_theta_deg = {120.0F, 240.0F, 0.0F};
    cfg.kinematics.plat_theta_deg = {120.0F, 240.0F, 0.0F};
    cfg.kinematics.servo_neutral_deg = {90.0F, 90.0F, 90.0F};
    cfg.kinematics.servo_dir = {-1, -1, -1};

    // Safety and command limits. The per-step limit is intentionally permissive
    // here because the physical rig uses the actuator manager's history/safety
    // path plus servo clamping, not an aggressive slew limiter.
    cfg.actuator.min_out = {0.0F, 0.0F, 0.0F};
    cfg.actuator.max_out = {180.0F, 180.0F, 180.0F};
    cfg.actuator.max_step = {999.0F, 999.0F, 999.0F};

    // Servo-driver baseline.
    cfg.servo.i2c_bus = 1;
    cfg.servo.pca9685_addr = 0x40U;
    cfg.servo.pwm_hz = 50.0F;
    cfg.servo.park_on_start = true;
    cfg.servo.park_on_stop = true;
    cfg.servo.log_every_n = 10;

    // Channel order matches the physical wiring on the rig.
    cfg.servo.ch[0] = {2, 500.0F, 2500.0F, 0.0F, 180.0F, kRigParkDeg, false};
    cfg.servo.ch[1] = {0, 500.0F, 2500.0F, 0.0F, 180.0F, kRigParkDeg, false};
    cfg.servo.ch[2] = {1, 500.0F, 2500.0F, 0.0F, 180.0F, kRigParkDeg, false};

    // Simulated camera defaults mirror the controller dimensions.
    cfg.simulated_camera.width = cfg.controller.image_width;
    cfg.simulated_camera.height = cfg.controller.image_height;
    cfg.simulated_camera.fps = 30;
    cfg.simulated_camera.moving_spot = true;
    cfg.simulated_camera.noise_std = 5.0F;
    cfg.simulated_camera.background = 20U;
    cfg.simulated_camera.spot_value = 240U;
    cfg.simulated_camera.spot_radius = 12;

    // Libcamera defaults mirror the controller dimensions.
    cfg.libcamera.width = cfg.controller.image_width;
    cfg.libcamera.height = cfg.controller.image_height;
    cfg.libcamera.fps = 30;

    // Manual-input mapping.
    cfg.manual_mapping.pot_supply_voltage = 3.3F;
    cfg.manual_mapping.invert_tilt = false;
    cfg.manual_mapping.invert_pan = false;
    cfg.manual_mapping.tilt_deadband_norm = 0.0F;
    cfg.manual_mapping.pan_deadband_norm = 0.0F;
    cfg.manual_mapping.max_manual_tilt_deg = 20.0F;
    cfg.manual_mapping.max_manual_pan_deg = 20.0F;

    // ADS1115 manual input hardware path.
    cfg.ads1115_manual.i2c_bus = 1;
    cfg.ads1115_manual.i2c_address = 0x48U;
    cfg.ads1115_manual.gpio_chip_index = 0;
    cfg.ads1115_manual.alert_rdy_gpio = 17U;
    cfg.ads1115_manual.tilt_channel = 0U;
    cfg.ads1115_manual.pan_channel = 1U;
    cfg.ads1115_manual.spare_channel = 0xFFU;
    cfg.ads1115_manual.full_scale_voltage = 4.096F;
    cfg.ads1115_manual.sample_rate_hz = 128U;

    // MPU6050 hardware path. WHO_AM_I expectation matches the currently used
    // live rig/runtime assumptions in this codebase.
    cfg.mpu6050.i2c_bus = 1;
    cfg.mpu6050.i2c_address = 0x68U;
    cfg.mpu6050.gpio_chip_index = 0;
    cfg.mpu6050.data_ready_gpio = 27U;
    cfg.mpu6050.who_am_i_expected = 0x70U;

    // Headless Linux event-loop tick rate.
    cfg.tick_hz = 30U;

    return cfg;
}

AppConfig defaultQtConfig() {
    AppConfig cfg = defaultConfig();

    // The Qt runtime uses the same baseline hardware/runtime configuration as
    // the headless path, but enables live IMU correction so the GUI reflects
    // real closed-loop behaviour.
    cfg.imu_feedback_mode = ImuFeedbackMode::Live;
    return cfg;
}

} // namespace solar::app