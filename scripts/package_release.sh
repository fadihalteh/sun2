#!/usr/bin/env bash
set -euo pipefail

BUILD_DIR="${1:-build}"
OUT_ZIP="${2:-solar-stewart-tracker-release.zip}"

rm -rf release_package
mkdir -p release_package/bin

cp "${BUILD_DIR}/solar_tracker" release_package/bin/

if [[ -f "${BUILD_DIR}/solar_tracker_qt" ]]; then
  cp "${BUILD_DIR}/solar_tracker_qt" release_package/bin/
fi

cp -r include release_package/
cp -r src release_package/
cp CMakeLists.txt release_package/

[[ -f README.md ]] && cp README.md release_package/
[[ -f LICENSE ]] && cp LICENSE release_package/
[[ -d docs ]] && cp -r docs release_package/

zip -r "${OUT_ZIP}" release_package