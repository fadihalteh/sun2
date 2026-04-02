# Reproducibility

## Purpose

This document explains how to reproduce the current repository snapshot from source. It is grounded in the actual contents of the snapshot: `CMakeLists.txt`, `.github/workflows/`, `scripts/`, `.gitmodules`, and the source tree under `src/`.

---

## 1. Source of Truth

The reproducible source of truth is:

| Path | Role |
|---|---|
| `src/` | All application and library source |
| `CMakeLists.txt` | Top-level build definition |
| `.gitmodules` | Submodule declarations |
| `external/` | Vendored and submodule dependencies |
| `scripts/` | Build, test, and release helper scripts |
| `.github/workflows/` | CI automation |
| `artefacts/latency.csv` | Captured runtime latency data |


---

## 2. External Dependencies

### Git Submodules

Two dependencies are declared as submodules in `.gitmodules`:

| Submodule | Path | URL |
|---|---|---|
| libgpiod_event_demo | `external/libgpiod_event_demo` | https://github.com/berndporr/libgpiod_event_demo |
| libcamera2opencv | `external/libcamera2opencv` | https://github.com/berndporr/libcamera2opencv |

### Vendored Dependency

`external/rpi_ads1115` is vendored directly in the repository (not a submodule) and requires no separate initialisation.

### System Packages

The CI workflow installs the following system packages on Ubuntu:

```bash
sudo apt-get install -y cmake ninja-build g++ pkg-config libgpiod-dev
```

Optional backends require additional packages:

- **OpenCV** — required for vision processing and the libcamera backend
- **libcamera** — required for the `LibcameraPublisher` camera backend (Pi only)
- **Qt5** — required for the `solar_tracker_qt` GUI target

---

## 3. CMake Build Options

All options and their defaults:

| Option | Default | Effect |
|---|---|---|
| `SOLAR_ENABLE_TESTS` | `ON` | Build and register test targets |
| `SOLAR_ENABLE_HW_TESTS` | `OFF` | Register hardware smoke tests with CTest |
| `SOLAR_ENABLE_QT` | `ON` | Build Qt GUI target if Qt5 is found |
| `SOLAR_TRY_LIBCAMERA` | `ON` | Probe for libcamera backend (requires OpenCV) |
| `SOLAR_TRY_OPENCV` | `ON` | Probe for OpenCV support |
| `SOLAR_ENABLE_COVERAGE` | `OFF` | Enable `--coverage` compile/link flags |

Optional features degrade gracefully — the build succeeds with reduced functionality if libcamera, OpenCV, or Qt5 are absent.

---

## 4. Clean Reproduction Procedure

```bash
# Step 1 — clone
git clone <repo-url>
cd Solar-Stewart-Tracker

# Step 2 — initialise submodules
git submodule update --init --recursive

# Step 3 — install system dependencies (Ubuntu / Debian)
sudo apt-get install -y cmake ninja-build g++ pkg-config libgpiod-dev
# Install libopencv-dev, libcamera-dev, qtbase5-dev as needed for optional backends

# Step 4 — configure
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release

# Step 5 — build
cmake --build build -j

# Step 6 — run software tests
ctest --test-dir build --output-on-failure

# Step 7 — run application (headless)
./build/solar_tracker

# Step 7 (alternative) — run Qt GUI target (if Qt5 was found at configure time)
./build/solar_tracker_qt
```

---

## 5. Helper Scripts

The `scripts/` directory contains the following:

| Script | Purpose |
|---|---|
| `scripts/test_core.sh [BUILD_DIR]` | Run CTest against a build directory |
| `scripts/test_pi_hw.sh [BUILD_DIR]` | Run hardware-labelled CTest checks on Pi |
| `scripts/run_latency.sh [BUILD_DIR] [CSV_PATH] [APP]` | Run the application with latency CSV export enabled |
| `scripts/package_release.sh [BUILD_DIR] [OUT_ZIP]` | Package binaries, source, and docs into a release zip |

> **Note:** The CI workflow references `scripts/build_core.sh` and `scripts/build_pi_debug.sh`, which are invoked during automated builds but are not present in this snapshot.

---

## 6. CI Automation

Four workflows are defined under `.github/workflows/`:

| Workflow | Trigger | Runner |
|---|---|---|
| `ci.yml` | Push/PR to `main`, `master`, `develop` | `ubuntu-latest` |
| `pi-hardware-tests.yml` | Manual dispatch | Self-hosted `linux/arm64` |
| `doxygen.yml` | See workflow file | See workflow file |
| `release.yml` | See workflow file | See workflow file |

The standard CI job (`ci.yml`) checks out with submodules, installs system packages, configures and builds via `scripts/build_core.sh`, then runs `scripts/test_core.sh`.

The Pi hardware workflow runs on a self-hosted runner, sets hardware test environment variables, and uploads the test log as an artifact.

---

## 7. Hardware Reproduction

For full hardware reproduction, the target should match the repository assumptions:

- Raspberry Pi (arm64)
- I2C bus at `/dev/i2c-1`
- PCA9685 PWM controller
- ADS1115 ADC (manual input potentiometers)
- MPU6050 IMU
- GPIO event support via `libgpiod`
- libcamera-compatible camera module

### Hardware Test Environment Variables

The `scripts/test_pi_hw.sh` script passes these to CTest:

| Variable | Default | Meaning |
|---|---|---|
| `SOLAR_RUN_CAMERA_HW_TESTS` | `1` | Enable camera hardware checks |
| `SOLAR_RUN_I2C_HW_TESTS` | `1` | Enable I2C hardware checks |
| `SOLAR_I2C_DEV` | `/dev/i2c-1` | I2C device path |

### Enabling Hardware Tests

```bash
cmake -S . -B build-pi -DCMAKE_BUILD_TYPE=Release -DSOLAR_ENABLE_HW_TESTS=ON
cmake --build build-pi -j
./scripts/test_pi_hw.sh build-pi
```

---

## 8. Latency Data Reproduction

The captured `artefacts/latency.csv` can be regenerated using:

```bash
./scripts/run_latency.sh build artefacts/latency.csv solar_tracker
```

This sets the `SOLAR_LATENCY_CSV` environment variable before launching the application, which causes the runtime `LatencyMonitor` to write per-frame measurements to the specified path on shutdown.

---

## 9. Non-Hardware Reproduction

If physical hardware is unavailable:

- the `SimulatedPublisher` camera backend allows full pipeline execution without a real camera
- all software tests in `test_core` remain runnable
- hardware-adjacent tests can be omitted by leaving `SOLAR_ENABLE_HW_TESTS` at its default (`OFF`)

---

## 10. Doxygen Documentation

A `Doxyfile` is present at the repository root. API documentation can be generated locally with:

```bash
doxygen Doxyfile
```

Automated documentation generation is handled by `.github/workflows/doxygen.yml`.

---

## 11. What This Document Does Not Claim

- the zip snapshot does not by itself prove GitHub issue or PR history
- physical actuator lag is not captured in the software latency CSV
- camera exposure time and mechanical dynamics are outside the software-only measurement scope
- the CI build scripts (`build_core.sh`, `build_pi_debug.sh`) are referenced by workflows but are not present in this snapshot

---

## 12. Summary

The repository is reproducible because submodule declarations, system dependencies, CMake options, build and test scripts, CI automation, hardware environment variables, and fallback execution paths are all visible in the snapshot. Each stage from source to running system has a documented, verifiable path.