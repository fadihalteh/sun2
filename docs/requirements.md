# Requirements

## Purpose

This document defines the requirements implemented in the current repository snapshot. All requirements are derived directly from the source — `src/`, `CMakeLists.txt`, `src/app/AppConfig.cpp`, and the hardware defaults in `src/app/AppConfig.hpp`.

---

## 1. Platform Requirements

| Requirement | Source |
|---|---|
| Linux userspace execution | `LinuxEventLoop`, `signalfd`, `timerfd`, `eventfd`, `poll()` |
| C++17 standard | `CMakeLists.txt`: `CMAKE_CXX_STANDARD 17` |
| CMake 3.16 or newer | `CMakeLists.txt`: `cmake_minimum_required(VERSION 3.16)` |
| Raspberry Pi (arm64) target for hardware mode | `pi-hardware-tests.yml`: `runs-on: [self-hosted, linux, arm64]` |
| I2C bus 1 available (`/dev/i2c-1`) | `AppConfig.cpp`: all hardware defaults use `i2c_bus = 1` |
| GPIO chip 0 available | `AppConfig.cpp`: `gpio_chip_index = 0` for ADS1115 and MPU6050 |
| `libgpiod` available | `CMakeLists.txt` CI installs `libgpiod-dev` unconditionally |
| Optional: Qt5 Widgets + Charts | `src/qt/CMakeLists.txt`: both components required for `solar_tracker_qt` |
| Optional: OpenCV + libcamera | `CMakeLists.txt`: libcamera backend requires both to be present |

---

## 2. Functional Requirements

### FR1 — Frame acquisition via abstract camera interface

The system shall acquire frames through `ICamera`, a pure virtual interface defining `registerFrameCallback()`, `start()`, `stop()`, and `isRunning()`.

Two concrete implementations shall be provided:
- `SimulatedPublisher` — always available; paces frames via `timerfd`-based `poll()` at a configurable rate (default 30 fps)
- `LibcameraPublisher` — available when both libcamera and OpenCV are found at configure time; wraps `libcamera2opencv`

The active backend is selected at construction time by `SystemFactory` based on `AppConfig::CameraBackend`.

### FR2 — Bright-target detection and centroid estimation

`SunTracker` shall process one `FrameEvent` and produce one `SunEstimate` containing:
- weighted centroid position (`cx`, `cy`) in pixels
- confidence value in range [0, 1]

Detection shall use a configurable brightness threshold (default 200, range 0–255 adjustable at runtime) and a minimum bright-pixel count (default 10). Supported pixel formats: `Gray8`, `RGB888`, `BGR888`, with padded-stride frames handled correctly.

### FR3 — Image-space error to platform setpoint conversion

`Controller` shall convert a `SunEstimate` into a `PlatformSetpoint` (tilt and pan in radians) using a proportional control law with:
- configurable gain per axis (default `k_pan = k_tilt = 0.8`)
- normalised deadband applied before the gain (default 0.0 in runtime config)
- output clamped to ±0.35 rad per axis
- estimates with confidence below `min_confidence` (default 0.4) produce a zero setpoint and suppress motion

### FR4 — Platform setpoint to actuator command via 3-RRS kinematics

`Kinematics3RRS` shall map a `PlatformSetpoint` (tilt, pan in radians) to an `ActuatorCommand` containing three servo targets in degrees, using the configured Stewart-platform geometry:
- base radius 0.20 m, platform radius 0.12 m
- home height 0.18 m, horn length 0.10 m, rod length 0.18 m
- base and platform joint angles at 120°, 240°, 0°

If the inverse kinematics solution is geometrically invalid, the system shall fall back to the last valid command (`CommandStatus::KinematicsFallbackLastValid`) or flag `CommandStatus::KinematicsInvalidConfig` and trigger FAULT.

### FR5 — Per-channel output safety conditioning

`ActuatorManager` shall apply to every `ActuatorCommand` before hardware output:
- per-channel clamping to configured output range (default 0°–180° per channel)
- per-channel slew-rate limiting to a configurable maximum step per update (default 999°, intentionally permissive — safety provided by servo clamping)
- history reset on startup to allow the first command to be fully saturated

### FR6 — Servo hardware output via PCA9685

`ServoDriver` shall output conditioned actuator commands to servos through the PCA9685 PWM controller at:
- I2C address `0x40`, bus 1
- 50 Hz PWM frequency
- pulse range 500–2500 µs mapped to 0°–180° servo range
- park position 41° applied on start and stop when `park_on_start`/`park_on_stop` are enabled

Three startup policies shall be supported: `LogOnly` (no hardware required), `PreferHardware` (hardware if available), `RequireHardware` (fail on missing hardware).

### FR7 — Manual control via potentiometer input

`ADS1115ManualInput` shall provide a hardware manual-input path using:
- ADS1115 ADC at I2C address `0x48`, bus 1
- ALERT/RDY pin on GPIO chip 0, pin 17 (falling-edge interrupt)
- 128 Hz sample rate, ±4.096 V full-scale
- tilt on channel 0, pan on channel 1
- 3.3 V pot supply voltage

`ManualInputMapper` shall map raw potentiometer voltages to normalised setpoints with configurable deadband and inversion per axis, clamping out-of-range inputs safely.

The manual command source shall be switchable at runtime between `ManualCommandSource::Pot` and `ManualCommandSource::Gui`.

### FR8 — IMU tilt measurement and optional correction

`Mpu6050Publisher` shall provide an IMU backend using:
- MPU6050 at I2C address `0x68`, bus 1
- data-ready pin on GPIO chip 0, pin 27 (GPIO-edge interrupt)
- WHO_AM_I identity check (expected value `0x70`) before use
- 8 startup samples discarded before data is delivered

`ImuTiltEstimator` shall compute tilt from accelerometer data. `ImuFeedbackMapper` shall apply a gain (default 0.12), deadband (1°), and maximum correction (1°) to produce a correction offset.

Three IMU feedback modes shall be supported:
- `Disabled` — IMU path inactive
- `Shadow` — IMU path active but correction not applied (observation only; default for headless runtime)
- `Live` — IMU correction applied to platform setpoint (default for Qt runtime)

IMU failure in Shadow mode shall degrade gracefully. IMU failure in Live mode shall trigger FAULT.

### FR9 — Headless CLI runtime

`LinuxEventLoop` shall provide a non-GUI runtime that blocks in a single `poll()` call multiplexing:
- `signalfd` for SIGINT/SIGTERM shutdown
- `timerfd` (CLOCK_MONOTONIC) for CLI tick at configurable rate (default 30 Hz)
- stdin for keyboard commands

`CliController` shall handle the following commands at runtime without restarting:

| Command | Effect |
|---|---|
| `manual` | Enter MANUAL state |
| `auto` | Exit to SEARCHING state |
| `set <tilt_rad> <pan_rad>` | Submit explicit manual setpoint |
| `threshold <0..255>` | Update vision detection threshold |
| `quit` / `exit` | Initiate clean shutdown |
| `help` | Print command list |

### FR10 — Qt GUI runtime

`solar_tracker_qt` shall provide a GUI runtime (built when Qt5 Widgets and Charts are available) with:
- AUTO, MANUAL, STOP buttons
- live camera frame preview
- mode and status labels (state, manual source, manual voltages, IMU readings)
- threshold `+`/`-` adjustment buttons
- pan and tilt sliders for GUI-driven manual setpoints
- "Use Pots" / "Use GUI" manual source switching
- servo command chart (3-channel actuator targets over time)
- ADS1115 voltage chart (tilt and pan potentiometer voltages over time)
- MPU6050 chart (accelerometer axes over time)

All plots and camera preview refresh at approximately 30 Hz via Qt timers.

### FR11 — Explicit runtime state machine

The runtime shall maintain an explicit state machine with eight states (IDLE, STARTUP, NEUTRAL, SEARCHING, TRACKING, MANUAL, STOPPING, FAULT) and well-defined transitions. No implicit state changes or hidden mode flags shall be used. State transitions shall be observable via `registerStateObserver()`.

### FR12 — Per-frame latency measurement

`LatencyMonitor` shall record monotonic timestamps at four pipeline stages per frame (capture, estimate, control, actuate) and compute:
- `L_vision` = t_estimate − t_capture
- `L_control` = t_control − t_estimate
- `L_actuation` = t_actuate − t_control
- `L_total` = t_actuate − t_capture

Summary statistics (average, minimum, maximum, jitter) shall be printed on shutdown. Raw per-frame data shall be exported to CSV when `SOLAR_LATENCY_CSV` is set.

### FR13 — Software test execution

The build system shall support a software test suite co-located with each module under `src/*/tests/`, runnable via CTest without physical hardware. Test executables shall support `--list` and `--run <name>` for individual case selection.

### FR14 — Hardware smoke test execution

The build system shall support hardware-adjacent tests via `-DSOLAR_ENABLE_HW_TESTS=ON`, runnable on a Raspberry Pi via `scripts/test_pi_hw.sh`. The Linux I2C hardware test shall support a skip path (`SKIP_RETURN_CODE 77`) when hardware is unavailable.

---

## 3. Architectural Requirements

### AR1 — Forward-only event-driven pipeline

The main processing path shall be a forward-only chain of transformations:

`ICamera → SunTracker → Controller → ManualImuCoordinator → Kinematics3RRS → ActuatorManager → ServoDriver`

No stage shall call back into an earlier stage. No stage shall own worker threads.

### AR2 — Blocking waits at all thread boundaries

Every thread in the system shall block using a proper Linux blocking primitive until work is available:

| Thread | Blocking mechanism |
|---|---|
| Control thread | `condition_variable::wait()` via `ThreadSafeQueue::wait_pop()` |
| Actuator thread | `condition_variable::wait()` via `ThreadSafeQueue::wait_pop()` |
| Application event loop | `poll()` on `signalfd`, `timerfd`, stdin |
| ADS1115 backend | GPIO ALERT/RDY falling-edge via `libgpiod` |
| MPU6050 backend | GPIO data-ready edge via `libgpiod` |
| SimulatedPublisher | `poll()` on `timerfd` + `eventfd` |

No polling loops, no `sleep()`-based pacing, and no busy-waiting are permitted in the processing path.

### AR3 — Bounded freshest-data queues

Inter-thread data transfer shall use `ThreadSafeQueue<T>` with bounded capacity and `push_latest()` policy:
- `frame_q_`: capacity 2, drops oldest frame if full
- `cmd_q_`: capacity 8, drops oldest command if full

This prevents stale backlog accumulation and ensures the consumer always processes the freshest available data.

### AR4 — Graceful optional feature degradation

All optional features shall degrade gracefully at configure time without breaking the build:
- libcamera absent → `SimulatedPublisher` selected automatically
- OpenCV absent → libcamera backend disabled even if libcamera is present
- Qt5 absent → `solar_tracker_qt` silently skipped
- IMU failure in Shadow mode → IMU disabled, runtime continues

### AR5 — Camera and hardware backend substitution

The camera backend shall be substitutable through the `ICamera` interface without modifying any downstream pipeline stage. The composition root (`SystemFactory`) shall be the only place where the concrete backend is selected.

### AR6 — Composition root isolation

`SystemFactory` shall be the sole location responsible for assembling the concrete runtime graph. `SystemManager` shall operate against interfaces only and shall not construct its own backends.

### AR7 — Centralised state control

All state transitions shall be performed exclusively by `SystemManager`. No other class shall set the runtime state. Transitions shall be explicit, logged, and observable via registered callbacks.

---

## 4. Non-Functional Requirements

### NFR1 — End-to-end software latency

Based on measured runtime evidence (`artefacts/latency.csv`, 419 frames):

| Metric | Measured |
|---|---|
| Average total latency | 8.37 ms |
| Worst-case total latency | 14.57 ms |
| Jitter (max − min) | 7.74 ms |
| Frame period at 30 Hz | 33 ms |

The system shall complete end-to-end processing within one frame period at 30 Hz under the measured conditions, with no systematic backlog accumulation.

### NFR2 — Dominant latency stage

Vision processing (`L_vision`) shall account for the dominant share of total latency. Control (`L_control`) and actuation (`L_actuation`) stages shall remain below 1 ms average.

### NFR3 — Reproducible build

The build shall be reproducible from source using only the declared dependencies (submodules, system packages, CMake options). No undeclared dependencies or implicit environment assumptions shall be required.

---

## 5. Safety Requirements

### SR1 — Park on start and stop

The servo driver shall apply a configurable park position (default 41°) on both `start()` and `stop()` when `park_on_start` and `park_on_stop` are enabled.

### SR2 — Safe shutdown under all active states

`stop()` shall be callable from any active state (SEARCHING, TRACKING, MANUAL, STARTUP, FAULT) and shall always complete the full shutdown sequence: camera stop → thread join → neutral command → driver stop → IDLE.

### SR3 — Kinematics fault propagation

An invalid kinematic solution shall immediately suppress actuator output and transition the system to FAULT. No invalid actuator target shall reach the servo driver.

### SR4 — Output clamping before hardware

Every actuator target shall pass through `ActuatorManager` (range clamp and slew limit) and `ServoDriver` (pulse-width clamp) before any hardware write. Raw kinematics output shall never reach the PCA9685 directly.