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

---

### FR2 — Bright-target detection and centroid estimation

`SunTracker` shall process one `FrameEvent` and produce one `SunEstimate` containing:
- weighted centroid position (`cx`, `cy`) in pixels
- confidence value in range [0, 1]

Detection shall use a configurable brightness threshold (default 200, range 0–255 adjustable at runtime) and a minimum bright-pixel count (default 10). Supported pixel formats: `Gray8`, `RGB888`, `BGR888`, with padded-stride frames handled correctly.

---

### FR3 — Image-space error to platform setpoint conversion

`Controller` shall convert a `SunEstimate` into a `PlatformSetpoint` (tilt and pan in radians) using a proportional control law with:
- configurable gain per axis (default `k_pan = k_tilt = 0.8`)
- normalised deadband applied before the gain (default 0.0 in runtime config)
- output clamped to ±0.35 rad per axis
- estimates with confidence below `min_confidence` (default 0.4) produce a zero setpoint and suppress motion

---

### FR4 — Platform setpoint to actuator command via 3-RRS kinematics

`Kinematics3RRS` shall map a `PlatformSetpoint` (tilt, pan in radians) to an `ActuatorCommand` containing three servo targets in degrees.

If the inverse kinematics solution is geometrically invalid, the system shall fall back to the last valid command or flag an invalid configuration and trigger FAULT.

---

### FR5 — Per-channel output safety conditioning

`ActuatorManager` shall apply to every `ActuatorCommand` before hardware output:
- per-channel clamping to configured output range (default 0°–180° per channel)
- per-channel slew-rate limiting (configurable; default permissive)
- history reset on startup to allow the first command to be fully applied

---

### FR6 — Servo hardware output via PCA9685

`ServoDriver` shall output conditioned actuator commands to servos through the PCA9685 PWM controller at:
- I2C address `0x40`, bus 1
- 50 Hz PWM frequency
- pulse range 500–2500 µs mapped to servo angle
- park position applied on start and stop when enabled

Three startup policies shall be supported: `LogOnly`, `PreferHardware`, and `RequireHardware`.

---

### FR7 — Manual control via potentiometer input

`ADS1115ManualInput` shall provide a manual-input path using:
- ADS1115 ADC at I2C address `0x48`, bus 1
- ALERT/RDY GPIO interrupt (edge-triggered)
- configurable sample rate and voltage range
- tilt and pan mapped from analog inputs

`ManualInputMapper` shall convert raw voltages to bounded setpoints with deadband and optional inversion.

---

### FR8 — IMU tilt measurement and optional correction

`Mpu6050Publisher` shall provide an IMU backend using I2C and a GPIO data-ready interrupt.

`ImuTiltEstimator` and `ImuFeedbackMapper` shall compute and optionally apply correction offsets.

Modes:
- Disabled
- Shadow (observe only)
- Live (apply correction)

Failure handling shall depend on mode.

---

### FR9 — Headless CLI runtime

`LinuxEventLoop` shall provide a runtime using a blocking `poll()` over:
- `signalfd` (signals)
- `timerfd` (tick)
- stdin (commands)

Commands shall include:
- `manual`, `auto`
- `set <tilt> <pan>`
- `threshold <value>`
- `quit`, `help`

---

### FR10 — Qt GUI runtime

`solar_tracker_qt` shall provide a GUI with:
- control buttons
- camera preview
- status display
- charts for actuator commands, ADC, and IMU

Updates occur at approximately 30 Hz using Qt timers.

---

### FR11 — Explicit runtime state machine

The system shall implement an explicit state machine:
IDLE, STARTUP, NEUTRAL, SEARCHING, TRACKING, MANUAL, STOPPING, FAULT.

Transitions shall be explicit and observable.

---

### FR12 — Per-frame latency measurement

`LatencyMonitor` shall record timestamps at key pipeline stages and compute per-frame latency metrics.

Summary statistics shall be printed on shutdown. CSV export shall be supported via environment configuration.

---

### FR13 — Software test execution

The build system shall support a software test suite runnable via CTest without requiring physical hardware.

Test binaries shall be executable directly. Additional command-line interfaces may exist but are not assumed to be uniform across all tests.

---

### FR14 — Hardware smoke test execution

The build system shall support hardware-adjacent tests via `-DSOLAR_ENABLE_HW_TESTS=ON`, runnable on a Raspberry Pi.

Hardware-dependent tests shall support safe skipping when hardware is unavailable.

---

## 3. Architectural Requirements

### AR1 — Forward-only event-driven pipeline

The processing path shall be:

`ICamera → SunTracker → Controller → ManualImuCoordinator → Kinematics3RRS → ActuatorManager → ServoDriver`

No stage shall call back into an earlier stage.

Stages may own worker threads where required for blocking I/O or event-driven execution. Threads shall have clear ownership, lifecycle, and no polling or sleep-based timing.

---

### AR2 — Blocking waits at all thread boundaries

All threads shall block on appropriate primitives until work is available:

| Thread | Mechanism |
|---|---|
| Control | `condition_variable` via queue |
| Actuator | `condition_variable` via queue |
| Event loop | `poll()` |
| Sensors | GPIO edge events |
| SimulatedPublisher | `timerfd` + `poll()` |

No polling loops or busy-waiting are permitted. Execution is driven by blocking I/O and event-triggered wake-ups.

---

### AR3 — Bounded freshest-data queues

Inter-thread communication shall use bounded queues with latest-wins behaviour:
- frame queue capacity: 2
- command queue capacity: 8

Oldest data is discarded when full.

---

### AR4 — Graceful optional feature degradation

Optional components shall not break the build:
- libcamera absent → fallback
- OpenCV absent → disable camera backend
- Qt absent → GUI skipped
- IMU failure handled by mode

---

### AR5 — Camera and backend substitution

Camera backend shall be substitutable via `ICamera` without modifying downstream stages.

---

### AR6 — Composition root isolation

`SystemFactory` shall act as the primary application assembly point.

Where practical, dependencies shall be injected into `SystemManager`. Internal components may still be constructed where required for lifecycle control.

---

### AR7 — Centralised state control

All state transitions shall be controlled by `SystemManager` and observable.

---

## 4. Non-Functional Requirements

### NFR1 — End-to-end latency

Measured runtime data demonstrates that total processing latency remains within one frame period at 30 Hz under tested conditions.

---

### NFR2 — Latency distribution

Vision processing is expected to dominate latency, with control and actuation remaining comparatively small.

---

### NFR3 — Reproducible build

The system shall build reproducibly using declared dependencies and documented steps.

---

## 5. Safety Requirements

### SR1 — Park on start and stop

Servos shall move to a safe park position during startup and shutdown.

---

### SR2 — Safe shutdown

Shutdown shall be safe from all states and complete all cleanup steps.

---

### SR3 — Kinematics fault handling

Invalid kinematics shall prevent actuator output and trigger FAULT.

---

### SR4 — Output safety enforcement

All actuator outputs shall be clamped and validated before reaching hardware.