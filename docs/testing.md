# Testing and Reliability Strategy

This document describes the current testing setup implemented in the repository source. Tests are co-located with their modules under `src/*/tests/` and wired into CTest via each module's own `CMakeLists.txt`.

The test approach is intentionally lightweight:

- a small custom runner is used instead of an external framework
- named checks are registered with CTest via each module's `CMakeLists.txt`
- core logic is exercised with deterministic inputs
- hardware-adjacent tests are separated behind an explicit build option
- fake I2C support is used where that improves automation

---

## 1. Testing Approach

The current test strategy is based on the following principles:

- keep core logic testable without physical hardware
- isolate vision, control, queueing, kinematics, and system state as deterministic software tests
- make hardware-adjacent tests opt-in via `-DSOLAR_ENABLE_HW_TESTS=ON`
- expose individual named cases through CTest
- avoid claiming that software-only tests prove full physical-system correctness

---

## 2. Test Framework

The repository does not use GoogleTest, Catch2, or another external framework. Instead it uses a small custom framework at `src/tests/support/`:

| File | Purpose |
|---|---|
| `src/tests/support/test_main.cpp` | Runner entry point |
| `src/tests/support/test_common.hpp` | `TEST_CASE(...)` registration and `REQUIRE(...)` assertion helper |
| `src/tests/support/FakeI2CDevice.hpp` | Fake I2C device for hardware-adjacent software tests |

Each test executable is built by linking `test_main.cpp` together with one or more test source files.

### Running an executable directly

```bash
# Run all tests compiled into an executable
./build/src/control/tests/test_control

# List all tests in an executable
./build/src/control/tests/test_control --list

# Run one specific test
./build/src/control/tests/test_control --run Controller_OutputClamped
```

With no arguments an executable runs every test compiled into it. This matters because a test can be compiled into an executable without being separately registered as a CTest entry.

---

## 3. Test Executables and Module Structure

Tests are co-located with source. Each module's `CMakeLists.txt` conditionally includes its `tests/` subdirectory.

### Always-built executables (`SOLAR_ENABLE_TESTS=ON`)

| Executable | Module | Source path |
|---|---|---|
| `test_common_core` | `src/common` | `src/common/tests/` |
| `test_vision` | `src/vision` | `src/vision/tests/` |
| `test_control` | `src/control` | `src/control/tests/` |
| `test_actuators` | `src/actuators` | `src/actuators/tests/` |
| `test_system` | `src/system` | `src/system/tests/` |

### Hardware-adjacent executables (`SOLAR_ENABLE_TESTS=ON` and `SOLAR_ENABLE_HW_TESTS=ON`)

| Executable | Module | Source path |
|---|---|---|
| `test_pca9685` | `src/actuators` | `src/actuators/tests/` |
| `test_servodriver` | `src/actuators` | `src/actuators/tests/` |
| `test_mpu6050_publisher` | `src/sensors` | `src/sensors/tests/` |
| `test_linux_i2c_hw` | `src/hal` | `src/hal/tests/` |

---

## 4. CTest Integration

CTest is used and named cases are registered with `add_test(...)` in each module's `CMakeLists.txt`, allowing failures to be reported at individual test-case level.

```bash
# Linux / single-config generators
ctest --test-dir build --output-on-failure

# Windows / multi-config generators
ctest --test-dir build -C Release --output-on-failure
```

### Test count

| Condition | CTest entries |
|---|---:|
| `SOLAR_ENABLE_TESTS=ON` only | 44 |
| `SOLAR_ENABLE_TESTS=ON` and `SOLAR_ENABLE_HW_TESTS=ON` | 61 |

---

## 5. CTest Cases by Module

### 5.1 SunTracker

Executable: `test_vision` — Source: `src/vision/tests/test_suntracker.cpp`

| CTest Case | What It Covers |
|---|---|
| `SunTracker_DetectsSingleBrightPixel` | Bright target detection |
| `SunTracker_NoBrightPixels_ProducesZeroConfidence` | Zero-confidence when no bright region exists |
| `SunTracker_WeightedCentroid_IsCorrect` | Weighted centroid on deterministic synthetic input |

### 5.2 Controller

Executable: `test_control` — Source: `src/control/tests/test_controller.cpp`

| CTest Case | What It Covers |
|---|---|
| `Controller_LowConfidence_NoMotion` | Confidence gating |
| `Controller_WithinDeadband_NoMotion` | Deadband behaviour |
| `Controller_OutsideDeadband_ProducesMotion` | Non-zero control response |
| `Controller_OutputClamped` | Output saturation/clamping |

### 5.3 Kinematics3RRS

Executable: `test_control` — Source: `src/control/tests/test_kinematics.cpp`

| CTest Case | What It Covers |
|---|---|
| `Kinematics3RRS_outputs_in_range_and_integer_like` | Output sanity and range |
| `Kinematics3RRS_continuity_small_setpoint_changes_small_output_changes` | Continuity for small input changes |
| `Kinematics3RRS_invalid_geometry_is_surfaced_explicitly` | Invalid-geometry surfacing |
| `Kinematics3RRS_falls_back_to_last_valid_when_subsequent_config_is_bad` | Fallback after a previously valid state |

### 5.4 ManualInputMapper

Executable: `test_control` — Source: `src/control/tests/test_manual_input_mapper.cpp`

| CTest Case | What It Covers |
|---|---|
| `ManualInputMapper_CentreVoltage_MapsToZeroCommand` | Centre mapping |
| `ManualInputMapper_LowAndHighVoltages_MapToMinusAndPlusOne` | Full-range mapping |
| `ManualInputMapper_InversionFlags_FlipDirections` | Inversion behaviour |
| `ManualInputMapper_Deadband_ZerosSmallInputsAndRescalesBeyondDeadband` | Deadband behaviour |
| `ManualInputMapper_OutOfRangeVoltages_AreClampedSafely` | Safe clamping of out-of-range inputs |

### 5.5 ImuFeedbackMapper

Executable: `test_control` — Source: `src/control/tests/test_imu_feedback_mapper.cpp`

| CTest Case | What It Covers |
|---|---|
| `ImuFeedbackMapper_ZeroTilt_ProducesZeroCorrection` | Zero-tilt response |
| `ImuFeedbackMapper_PositiveTilt_ProducesNegativeCorrection` | Correction sign |
| `ImuFeedbackMapper_NegativeTilt_ProducesPositiveCorrection` | Symmetry across tilt direction |
| `ImuFeedbackMapper_RuntimeGainChange_TakesEffectImmediately` | Immediate effect of runtime gain changes |

### 5.6 ImuTiltEstimator

Executable: `test_control` — Source: `src/control/tests/test_imu_tilt_estimator.cpp`

| CTest Case | What It Covers |
|---|---|
| `ImuTiltEstimator_InvalidSample_ReturnsZero` | Invalid sample handling |
| `ImuTiltEstimator_LevelGravity_ReturnsZeroTilt` | Level orientation |
| `ImuTiltEstimator_PositiveAx_WithGravity_ReturnsPositiveTilt` | Positive tilt reconstruction |
| `ImuTiltEstimator_NegativeAx_WithGravity_ReturnsNegativeTilt` | Negative tilt reconstruction |
| `ImuTiltEstimator_DegenerateDenominator_ReturnsZero` | Degenerate denominator protection |

### 5.7 ActuatorManager

Executable: `test_actuators` — Source: `src/actuators/tests/test_actuatormanager.cpp`

| CTest Case | What It Covers |
|---|---|
| `ActuatorManager_first_command_is_saturated_without_history_limit` | First-command behaviour |
| `ActuatorManager_subsequent_commands_are_rate_limited_per_channel` | Per-channel rate limiting |
| `ActuatorManager_saturation_happens_before_history_update` | Saturation ordering |
| `ActuatorManager_resetHistory_disables_slew_limit_for_next_command` | History reset behaviour |
| `ActuatorManager_zero_or_negative_step_holds_previous_value` | Behaviour when allowed step is zero or negative |

### 5.8 ThreadSafeQueue

Executable: `test_common_core` — Source: `src/common/tests/test_threadsafequeue.cpp`

| CTest Case | What It Covers |
|---|---|
| `ThreadSafeQueue_FIFO_basic` | FIFO ordering |
| `ThreadSafeQueue_bounded_push_strict_rejects_when_full` | Strict bounded insertion |
| `ThreadSafeQueue_bounded_push_latest_drops_oldest` | Freshest-value overwrite semantics |
| `ThreadSafeQueue_wait_pop_blocks_then_wakes` | Blocking wake-up behaviour |
| `ThreadSafeQueue_stop_unblocks_waiters_and_returns_nullopt_when_empty` | Stop/unblock behaviour |

### 5.9 LatencyMonitor

Executable: `test_common_core` — Source: `src/common/tests/test_latency_monitor.cpp`

| CTest Case | What It Covers |
|---|---|
| `LatencyMonitor_accepts_ordered_timestamps_and_prints` | Ordered timestamp handling |
| `LatencyMonitor_handles_out_of_order_calls_without_crashing` | Out-of-order robustness |
| `LatencyMonitor_prunes_inflight_frames_under_pressure_without_crashing` | In-flight frame pruning under pressure |

> **Note:** A fourth test (`LatencyMonitor_reports_jitter_over_multiple_records`) is compiled into `test_common_core` but is not individually registered as a CTest entry.

### 5.10 SystemManager

Executable: `test_system`  
Sources: `src/system/tests/test_systemmanager_statemachine.cpp`, `src/system/tests/test_systemmanager_imu_shadow.cpp`

| CTest Case | What It Covers |
|---|---|
| `SystemManager_start_to_searching_then_tracking_on_bright_frame` | Startup progression and tracking-state transition |
| `SystemManager_manual_mode_emits_commands` | Manual mode command emission |
| `SystemManager_start_with_null_camera_enters_fault_and_fails` | Fault entry on null camera |
| `SystemManager_start_when_camera_start_fails_enters_fault` | Fault entry on camera start failure |
| `SystemManager_ImuShadowMode_StartsWithoutImuBackendWhenDisabled` | IMU shadow-mode startup behaviour |
| `SystemManager_ImuShadowMode_DoesNotPreventManualModeTransitions` | IMU shadow-mode interaction with manual transitions |

---

## 6. Hardware-Adjacent Tests (`SOLAR_ENABLE_HW_TESTS=ON`)

### 6.1 PCA9685

Executable: `test_pca9685` — Source: `src/actuators/tests/test_pca9685.cpp`

| CTest Case | What It Covers |
|---|---|
| `PCA9685_default_constructor_is_not_started` | Safe default construction |
| `PCA9685_set_pulse_us_fails_before_start` | Pre-start failure handling |
| `PCA9685_invalid_channel_rejected_even_if_not_started` | Channel validation |
| `PCA9685_stop_is_safe_when_not_started` | Safe stop before startup |

### 6.2 ServoDriver

Executable: `test_servodriver` — Source: `src/actuators/tests/test_servodriver.cpp`

| CTest Case | What It Covers |
|---|---|
| `ServoDriver_log_only_mode_starts_without_hardware` | Startup policy across hardware modes |
| `ServoDriver_require_hardware_fails_fast_when_unavailable` | Fast failure on missing required hardware |
| `ServoDriver_prefer_hardware_falls_back_to_log_only_when_unavailable` | Fallback to log-only mode |
| `ServoDriver_require_hardware_with_injected_pca_enters_hardware_mode` | Injected PCA path |
| `ServoDriver_apply_while_stopped_writes_nothing` | No writes while stopped |
| `ServoDriver_start_parks_to_neutral_when_enabled` | Parking on start |
| `ServoDriver_stop_parks_to_neutral_when_enabled` | Parking on stop |
| `ServoDriver_apply_clamps_and_writes_channels` | Command clamping and channel writes |
| `ServoDriver_inverted_channel_maps_correctly` | Inverted channel mapping |

### 6.3 Mpu6050Publisher

Executable: `test_mpu6050_publisher` — Source: `src/sensors/tests/test_mpu6050_publisher.cpp`

| CTest Case | What It Covers |
|---|---|
| `Mpu6050Publisher_StartFailsWithoutCallback` | Callback requirement |
| `Mpu6050Publisher_StartFailsIfWhoAmIDoesNotMatch` | Identity check |
| `Mpu6050Publisher_StartInitialisesDevice_WhenWhoAmIMatches` | Correct initialisation path |

### 6.4 Linux I2C Hardware Check

Executable: `test_linux_i2c_hw` — Source: `src/hal/tests/test_linux_i2c_hw.cpp`

| CTest Case | Labels | Notes |
|---|---|---|
| `LinuxI2C_PCA9685_hw_init_and_write` | `hw;i2c;integration` | Skips cleanly (`SKIP_RETURN_CODE 77`) when hardware is unavailable |

---

## 7. What the Current Tests Demonstrate

- bright-target detection and centroid logic
- controller gating, deadband, and output clamping
- bounded queue semantics and blocking wake-up behaviour
- actuator command limiting and history behaviour
- kinematics validity, continuity, and fallback behaviour
- latency-monitor robustness
- system-manager startup, manual-mode, and selected fault paths
- manual input mapping
- IMU tilt estimation and IMU feedback mapping
- selected startup and mapping behaviour of PCA9685, ServoDriver, and MPU6050 publisher (when hardware-adjacent tests are enabled)

---

## 8. What the Current Tests Do Not Prove

- full physical motion correctness of the assembled mechanism
- long-duration reliability on real hardware
- complete actuator mechanical response
- full end-to-end camera-to-motion physical-loop validation
- hard realtime guarantees under all scheduling conditions
- exhaustive handling of all hardware fault scenarios

---

## 9. Practical Commands

```bash
# Run all CTest-registered checks
ctest --test-dir build --output-on-failure

# Run individual module executables
./build/src/vision/tests/test_vision --list
./build/src/control/tests/test_control --run Controller_OutputClamped
./build/src/common/tests/test_common_core --run ThreadSafeQueue_wait_pop_blocks_then_wakes
./build/src/actuators/tests/test_actuators --run ActuatorManager_resetHistory_disables_slew_limit_for_next_command
./build/src/system/tests/test_system --run SystemManager_manual_mode_emits_commands

# Enable and build hardware-adjacent tests
cmake -S . -B build -DSOLAR_ENABLE_HW_TESTS=ON
cmake --build build -j

# List hardware-adjacent executables
./build/src/actuators/tests/test_pca9685 --list
./build/src/actuators/tests/test_servodriver --list
./build/src/sensors/tests/test_mpu6050_publisher --list
```

---

## 10. Maintenance Note

This document matches each module's `CMakeLists.txt` under `src/*/tests/`. If the test inventory changes, this document must be updated at the same time. The following distinctions must remain explicit:

- which executable a test is compiled into
- which tests are individually registered as CTest entries
- which modules require `SOLAR_ENABLE_HW_TESTS=ON` to build their tests