# Dependencies

## Purpose

This document lists all dependencies required to build and run the current repository snapshot. All hardware defaults are sourced directly from `src/app/AppConfig.cpp` and `src/app/AppConfig.hpp`.

---

## 1. Core Build Requirements

From `CMakeLists.txt`:

| Requirement | Minimum |
|---|---|
| CMake | 3.16 |
| C++ standard | C++17 |
| Compiler flags | `-Wall -Wextra -Wpedantic` (GCC/Clang) |
| Threads | `find_package(Threads REQUIRED)` |
| pkg-config | `find_package(PkgConfig REQUIRED)` |

```bash
sudo apt-get install -y cmake ninja-build g++ pkg-config git
```

---

## 2. Required Runtime Dependency

### libgpiod

Required unconditionally. Used by the ADS1115 manual input and MPU6050 IMU backends for GPIO edge event handling via `libgpiod_event_demo`.

```bash
sudo apt-get install -y libgpiod-dev
```

---

## 3. Optional Feature Dependencies

All optional features are probed at configure time and degrade gracefully if absent. The build always succeeds.

### Qt5 — GUI target (`solar_tracker_qt`)

Requires **both** `Widgets` and `Charts` components. If either is missing, `solar_tracker_qt` is silently skipped.

```bash
sudo apt-get install -y qtbase5-dev libqt5charts5-dev
```

Controlled by: `-DSOLAR_ENABLE_QT=ON` (default)

### OpenCV — vision processing and libcamera bridge

Required for image frame processing in `SunTracker` and as a prerequisite for the libcamera backend.

```bash
sudo apt-get install -y libopencv-dev
```

Controlled by: `-DSOLAR_TRY_OPENCV=ON` (default)

### libcamera — Raspberry Pi camera backend

Enables the `LibcameraPublisher` camera backend. **Requires OpenCV to also be present** — the CMake configuration explicitly checks for both:

```
if(LIBCAMERA_FOUND AND SOLAR_HAVE_OPENCV) → backend enabled
if(LIBCAMERA_FOUND AND NOT SOLAR_HAVE_OPENCV) → backend disabled
```

```bash
sudo apt-get install -y libcamera-dev
```

Controlled by: `-DSOLAR_TRY_LIBCAMERA=ON` (default)

> Installing libcamera without OpenCV will not enable the libcamera backend.

---

## 4. External Repository Dependencies

### Git Submodules

This repository currently declares **three** git submodules in `.gitmodules`:

| Submodule | Path | Role in this repository |
|---|---|---|
| libgpiod_event_demo | `external/libgpiod_event_demo` | GPIO edge-event callback support used by the ADS1115 manual-input and MPU6050 data-ready paths |
| libcamera2opencv | `external/libcamera2opencv` | libcamera-to-OpenCV bridge used only when libcamera and OpenCV support are both enabled |

```bash
git submodule update --init --recursive
```
---

## 5. Linux Kernel Interfaces

The following kernel interfaces are used at runtime:

| Interface | Used by |
|---|---|
| `/dev/i2c-1` | PCA9685, ADS1115, MPU6050 |
| GPIO chip 0 | ADS1115 ALERT/RDY, MPU6050 data-ready |
| `poll()` | `LinuxEventLoop`, `SimulatedPublisher` |
| `signalfd` | `LinuxEventLoop` — SIGINT/SIGTERM shutdown |
| `timerfd` (CLOCK_MONOTONIC) | `LinuxEventLoop` (CLI tick), `SimulatedPublisher` (frame pacing) |
| `eventfd` | `SimulatedPublisher` — clean stop wakeup |

---

## 6. Hardware Dependencies and Default Configuration

All defaults are from `src/app/AppConfig.cpp` (`defaultConfig()`).

### PCA9685 — Servo Driver

| Parameter | Default |
|---|---|
| I2C bus | 1 (`/dev/i2c-1`) |
| I2C address | `0x40` |
| PWM frequency | 50 Hz |
| Pulse range | 500 – 2500 µs |
| Servo range | 0° – 180° |
| Park angle | 41° (all three channels) |
| Park on start | enabled |
| Park on stop | enabled |
| Channel mapping | ch0 → PCA pin 2, ch1 → PCA pin 0, ch2 → PCA pin 1 |

### ADS1115 — Manual Input (Potentiometers)

| Parameter | Default |
|---|---|
| I2C bus | 1 (`/dev/i2c-1`) |
| I2C address | `0x48` |
| GPIO chip | 0 |
| ALERT/RDY GPIO pin | 17 |
| Tilt channel | 0 |
| Pan channel | 1 |
| Full-scale voltage | ±4.096 V |
| Sample rate | 128 Hz |
| Pot supply voltage | 3.3 V |

Manual input is **enabled by default** in `defaultConfig()`.

### MPU6050 — IMU

| Parameter | Default |
|---|---|
| I2C bus | 1 (`/dev/i2c-1`) |
| I2C address | `0x68` |
| GPIO chip | 0 |
| Data-ready GPIO pin | 27 |
| WHO\_AM\_I expected | `0x70` |
| Startup discard samples | 8 |
| Accel sensitivity | 16384 LSB/g (±2g range) |
| Gyro sensitivity | 131 LSB/°s (±250°/s range) |

IMU is **enabled by default** in `defaultConfig()` in Shadow mode. Qt runtime uses Live mode.

### Camera

| Backend | Condition | Default resolution | Default FPS |
|---|---|---|---|
| `SimulatedPublisher` | Always available | 640 × 480 | 30 |
| `LibcameraPublisher` | Requires libcamera **and** OpenCV | 640 × 480 | 30 |

When built with libcamera support, `defaultConfig()` selects `Libcamera` and sets `ServoDriver::StartupPolicy::RequireHardware`. Without libcamera, it selects `Simulated` and `LogOnly`.

### Stewart Platform Kinematics

| Parameter | Default |
|---|---|
| Base radius | 0.20 m |
| Platform radius | 0.12 m |
| Home height | 0.18 m |
| Horn length | 0.10 m |
| Rod length | 0.18 m |
| Base joint angles | 120°, 240°, 0° |
| Platform joint angles | 120°, 240°, 0° |
| Servo neutral positions | 90°, 90°, 90° |
| Servo directions | −1, −1, −1 |

---

## 7. Hardware Validation

Before running on real hardware, verify I2C and GPIO availability:

```bash
# Verify I2C devices are visible on bus 1
# Expected: PCA9685 at 0x40, ADS1115 at 0x48, MPU6050 at 0x68
i2cdetect -y 1

# List available GPIO chips and lines
gpioinfo
```

---

## 8. CMake Option Reference

| Option | Default | Effect |
|---|---|---|
| `SOLAR_ENABLE_TESTS` | `ON` | Build test targets |
| `SOLAR_ENABLE_HW_TESTS` | `OFF` | Register hardware smoke tests with CTest |
| `SOLAR_ENABLE_QT` | `ON` | Build `solar_tracker_qt` if Qt5 found |
| `SOLAR_TRY_LIBCAMERA` | `ON` | Probe for libcamera (also requires OpenCV) |
| `SOLAR_TRY_OPENCV` | `ON` | Probe for OpenCV |
| `SOLAR_ENABLE_COVERAGE` | `OFF` | Enable `--coverage` compile/link flags |

---

## 9. Full Install Command

```bash
# Required
sudo apt-get install -y cmake ninja-build g++ pkg-config git libgpiod-dev

# Optional — Qt GUI
sudo apt-get install -y qtbase5-dev libqt5charts5-dev

# Optional — vision and libcamera (both needed together)
sudo apt-get install -y libopencv-dev libcamera-dev

# Optional — hardware validation
sudo apt-get install -y i2c-tools
```