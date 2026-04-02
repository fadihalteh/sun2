#!/usr/bin/env bash
set -euo pipefail

BUILD_DIR="${1:-build-pi}"

export SOLAR_RUN_CAMERA_HW_TESTS="${SOLAR_RUN_CAMERA_HW_TESTS:-1}"
export SOLAR_RUN_I2C_HW_TESTS="${SOLAR_RUN_I2C_HW_TESTS:-1}"
export SOLAR_I2C_DEV="${SOLAR_I2C_DEV:-/dev/i2c-1}"

ctest --test-dir "${BUILD_DIR}" -L hw --output-on-failure