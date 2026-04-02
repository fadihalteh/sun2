# Project Overview

## Project Identity

This repository is an ENG5220 real-time embedded programming project implementing a Stewart-platform solar tracker on Raspberry Pi/Linux userspace.

The system observes a camera stream, locates the brightest target region, converts that image-space error into platform tilt and pan demands, maps those demands into three actuator commands, applies final output conditioning, and drives the platform through the servo hardware path.

---

## End-to-End Behaviour

The implemented runtime pipeline is:

```text
Camera → SunTracker → Controller → Kinematics3RRS → ActuatorManager → ServoDriver
```

This pipeline is visible directly in the staged source tree:
- `src/vision/SunTracker.cpp`
- `src/control/Controller.cpp`
- `src/control/Kinematics3RRS.cpp`
- `src/actuators/ActuatorManager.cpp`
- `src/actuators/ServoDriver.cpp`

and is orchestrated in:
- `src/system/SystemManager.cpp`

---

## What the Current Snapshot Supports

The current repository snapshot supports:

- automatic tracking
- manual mode
- IMU observation or live correction path
- headless Linux event-loop execution
- Qt GUI execution
- simulated camera fallback
- software unit tests
- Raspberry Pi hardware smoke-test workflow
- latency CSV generation
- Doxygen generation
- release packaging workflow

These claims are grounded in the actual files present in the zip.

---

## Main Repository Subsystems

### Runtime and composition
- `src/main.cpp`
- `src/qt/main_qt.cpp`
- `src/app/Application.cpp`
- `src/app/LinuxEventLoop.cpp`
- `src/app/CliController.cpp`
- `src/app/SystemFactory.cpp`
- `src/app/AppConfig.cpp`

### Vision and control
- `src/vision/SunTracker.cpp`
- `src/control/Controller.cpp`
- `src/control/Kinematics3RRS.cpp`
- `src/control/ManualInputMapper.cpp`
- `src/control/ImuFeedbackMapper.cpp`
- `src/control/ImuTiltEstimator.cpp`
- `src/control/ManualImuCoordinator.cpp`

### Actuation
- `src/actuators/ActuatorManager.cpp`
- `src/actuators/ServoDriver.cpp`
- `src/actuators/PCA9685.cpp`

### Hardware/backend support
- `src/sensors/SimulatedPublisher.cpp`
- `src/sensors/LibcameraPublisher.cpp`
- `src/sensors/manual/ADS1115ManualInput.cpp`
- `src/sensors/imu/Mpu6050Publisher.cpp`
- `src/hal/LinuxI2CDevice.cpp`

### System orchestration
- `src/system/SystemManager.cpp`
- `include/system/TrackerState.hpp`

### GUI
- `src/qt/MainWindow.cpp`
- `src/qt/MainWindow.hpp`

---

## Build Targets Visible in the Snapshot

The top-level `CMakeLists.txt` declares these visible targets:

- `cam2opencv`
- `solar_tracker_core`
- `solar_tracker`
- `solar_tracker_qt`

The important application targets are:
- `solar_tracker`
- `solar_tracker_qt`

The main library target is:
- `solar_tracker_core`

---

## Runtime Modes and Backends

Configuration in `include/app/AppConfig.hpp` and `src/app/AppConfig.cpp` shows support for:

- `CameraBackend::Simulated`
- `CameraBackend::Libcamera`
- `StartupMode::Auto`
- `StartupMode::Manual`
- `ManualInputBackend::ADS1115`
- `ImuBackend::Mpu6050Gpio`
- `ImuFeedbackMode::Disabled`
- `ImuFeedbackMode::Shadow`
- `ImuFeedbackMode::Live`

---

## Honest Limits of This Snapshot

To keep this overview honest:

- the zip snapshot does not itself prove GitHub issue or PR history,
- `src/qt/main_qt.cpp` still contains runtime configuration policy that is broader than an ideal minimal `main()`,
- `SystemManager` is broader than an absolutely minimal orchestration class,
- the simulation backend exists as a support path and should not be confused with the main hardware proof path.

---

## Summary

This project is a real-time, event-driven Stewart-platform solar tracker implemented as a staged Linux userspace system. The repository snapshot already contains the code, tests, scripts, workflows, and latency artifact required to describe a serious ENG5220 submission rather than a one-file prototype.
