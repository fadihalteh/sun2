# Build and Run

## Purpose

This document gives an exact build-and-run procedure based on the current repository snapshot. All targets, scripts, and CMake options are verified against `CMakeLists.txt` and `.github/workflows/`.

---

## 1. Install Dependencies

### Required (all builds)

```bash
sudo apt-get update
sudo apt-get install -y cmake ninja-build g++ pkg-config libgpiod-dev
```

### Optional — Qt GUI target

Requires both `Widgets` and `Charts` components. If either is missing, `solar_tracker_qt` will not be built.

```bash
sudo apt-get install -y qtbase5-dev libqt5charts5-dev
```

### Optional — libcamera backend

The libcamera backend only activates when **both** libcamera and OpenCV are present. Installing libcamera alone is not sufficient.

```bash
sudo apt-get install -y libopencv-dev libcamera-dev
```

### Optional — hardware validation tools

```bash
sudo apt-get install -y i2c-tools
```

---

## 2. Clone and Initialise Submodules

```bash
git clone <repo-url>
cd Solar-Stewart-Tracker
git submodule update --init --recursive
```

Two submodules are declared in `.gitmodules`:

- `external/libgpiod_event_demo`
- `external/libcamera2opencv`

`external/rpi_ads1115` is vendored directly and requires no separate initialisation.

---

## 3. Configure

### Standard release build
```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
```

### With Ninja (faster incremental builds)
```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
```

### Disable optional features explicitly
```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
    -DSOLAR_ENABLE_QT=OFF \
    -DSOLAR_TRY_LIBCAMERA=OFF \
    -DSOLAR_TRY_OPENCV=OFF
```

### With hardware tests enabled
```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DSOLAR_ENABLE_HW_TESTS=ON
```

All CMake options and their defaults:

| Option | Default | Effect |
|---|---|---|
| `SOLAR_ENABLE_TESTS` | `ON` | Build test targets |
| `SOLAR_ENABLE_HW_TESTS` | `OFF` | Register hardware smoke tests with CTest |
| `SOLAR_ENABLE_QT` | `ON` | Build `solar_tracker_qt` if Qt5 found |
| `SOLAR_TRY_LIBCAMERA` | `ON` | Probe for libcamera (also requires OpenCV) |
| `SOLAR_TRY_OPENCV` | `ON` | Probe for OpenCV |
| `SOLAR_ENABLE_COVERAGE` | `OFF` | Enable `--coverage` compile/link flags |

---

## 4. Build

```bash
cmake --build build -j
```

### User-facing executables

| Target | Condition |
|---|---|
| `solar_tracker` | Always built |
| `solar_tracker_qt` | Built only if `Qt5 Widgets` and `Qt5 Charts` are found |

### Key internal library targets

| Target | Source |
|---|---|
| `solar_common` | `src/common/` |
| `solar_hal` | `src/hal/` |
| `solar_control` | `src/control/` |
| `solar_actuators` | `src/actuators/` |
| `solar_sensors` | `src/sensors/` |
| `solar_vision` | `src/vision/` |
| `solar_system` | `src/system/` |
| `solar_app` | `src/app/` |
| `cam2opencv` | Built only if libcamera **and** OpenCV are both found |

---

## 5. Run Software Tests

```bash
ctest --test-dir build --output-on-failure
```

Or using the repository helper script:

```bash
./scripts/test_core.sh build
```

To run one specific test directly:

```bash
./build/tests/test_actuators --run ActuatorManager_first_command_is_saturated_without_history_limit
./build/tests/test_actuators --list
```

---

## 6. Run the Application

### Headless runtime

```bash
./build/solar_tracker
```

### Qt GUI runtime

Only available if Qt5 was found at configure time:

```bash
./build/solar_tracker_qt
```

---

## 7. Capture Latency Data

```bash
./scripts/run_latency.sh build artefacts/latency.csv solar_tracker
```

This sets `SOLAR_LATENCY_CSV` before launching the application. On shutdown, the `LatencyMonitor` writes per-frame timing data to the specified path.

---

## 8. Hardware Tests (Raspberry Pi)

Hardware tests require a self-hosted Pi runner and are triggered manually in CI via `pi-hardware-tests.yml`. The build and run scripts referenced by that workflow (`build_pi_debug.sh`) are not present in this snapshot.

To run hardware-registered CTest checks on a Pi with a built tree:

```bash
cmake -S . -B build-pi -DCMAKE_BUILD_TYPE=Debug -DSOLAR_ENABLE_HW_TESTS=ON
cmake --build build-pi -j
./scripts/test_pi_hw.sh build-pi
```

The `test_pi_hw.sh` script sets these environment variables before invoking CTest:

| Variable | Default | Meaning |
|---|---|---|
| `SOLAR_RUN_CAMERA_HW_TESTS` | `1` | Enable camera hardware checks |
| `SOLAR_RUN_I2C_HW_TESTS` | `1` | Enable I2C hardware checks |
| `SOLAR_I2C_DEV` | `/dev/i2c-1` | I2C device path |

Hardware validation before running:

```bash
i2cdetect -y 1    # verify PCA9685, ADS1115, MPU6050 are visible on the bus
gpioinfo          # verify GPIO chip and pin availability
```

---

## 9. Package a Release

```bash
./scripts/package_release.sh build solar-stewart-tracker-release.zip
```

This copies `solar_tracker` (and `solar_tracker_qt` if present), `src/`, `CMakeLists.txt`, `README.md`, `LICENSE`, and `docs/` into a `release_package/` directory and zips it.

---

## 10. Quick-Start Summary

```bash
# Clone
git clone <repo-url>
cd Solar-Stewart-Tracker
git submodule update --init --recursive

# Install minimum dependencies
sudo apt-get install -y cmake ninja-build g++ pkg-config libgpiod-dev

# Configure and build
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j

# Test
ctest --test-dir build --output-on-failure

# Run
./build/solar_tracker
```