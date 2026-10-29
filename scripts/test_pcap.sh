#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${SDK_TEST_IMAGE:-yili-lidar:release-builder}"
V192_PCAP="${V192_PCAP:-$ROOT_DIR/../testdata/v192_demo.pcap}"
P60_IMU_PCAP="${P60_IMU_PCAP:-$ROOT_DIR/../testdata/p60_imu_demo.pcap}"
BUILD_DIR="${BUILD_DIR:-}"
RESULT_DIR="${RESULT_DIR:-$ROOT_DIR/out/test-results}"
SAVE_FRAMES="${SAVE_FRAMES:-3}"
USE_DOCKER=0

case "${1:-}" in
  --docker) USE_DOCKER=1; shift ;;
  --no-docker) USE_DOCKER=0; shift ;;
  "") ;;
  *) echo "usage: $0 [--docker|--no-docker]" >&2; exit 2 ;;
esac
if [[ $# -ne 0 ]]; then
  echo "usage: $0 [--docker|--no-docker]" >&2
  exit 2
fi

[[ -f "$V192_PCAP" ]] || { echo "PCAP not found: $V192_PCAP" >&2; exit 2; }
[[ -f "$P60_IMU_PCAP" ]] || { echo "PCAP not found: $P60_IMU_PCAP" >&2; exit 2; }
mkdir -p "$RESULT_DIR/sdk_output"

ARCH="$(uname -m)"
case "$ARCH" in
  x86_64|amd64) LIB_ARCH_DIR=x86-64 ;;
  aarch64|arm64) LIB_ARCH_DIR=arm64 ;;
  *) echo "Unsupported native architecture: $ARCH" >&2; exit 2 ;;
esac

run_tests() {
  local root="$1"
  local v192="$2"
  local p60="$3"
  local result_dir="$4"
  local build_dir="${BUILD_DIR:-$(mktemp -d /tmp/lidar_sdk_pcap_build.XXXXXX)}"

  cmake -S "$root/lidar_driver_sdk" -B "$build_dir" \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_DEMO=OFF \
    -DBUILD_PCAP_TEST=ON
  cmake --build "$build_dir" --target lidar_sdk_pcap_test -j"$(nproc)"

  mkdir -p "$result_dir/sdk_output"
  export LD_LIBRARY_PATH="$root/lidar_driver_sdk/lib/$LIB_ARCH_DIR:$build_dir:${LD_LIBRARY_PATH:-}"
  export LIDAR_SDK_TEST_CALIB_DIR="$root/lidar_driver_sdk/resources/calibration"
  export LIDAR_SDK_TEST_SAVE_FRAMES="$SAVE_FRAMES"

  set +e
  LIDAR_SDK_TEST_OUTPUT_DIR="$result_dir/sdk_output" \
    "$build_dir/lidar_sdk_pcap_test" V192 "$v192" 5 \
    2>&1 | tee "$result_dir/sdk_v192.log"
  local r1=${PIPESTATUS[0]}
  LIDAR_SDK_TEST_OUTPUT_DIR="$result_dir/sdk_output" \
    "$build_dir/lidar_sdk_pcap_test" P60_IMU "$p60" 10 \
    2>&1 | tee "$result_dir/sdk_p60_imu.log"
  local r2=${PIPESTATUS[0]}
  set -e

  if grep -Eiq 'SIGSEGV|segmentation fault|exit code -11' \
      "$result_dir/sdk_v192.log" "$result_dir/sdk_p60_imu.log"; then
    echo "SDK PCAP test failed: crash detected" >&2
    return 1
  fi
  if (( r1 != 0 || r2 != 0 )); then
    echo "SDK PCAP test failed: V192=$r1 P60_IMU=$r2" >&2
    return 1
  fi
  echo "SDK PCAP tests passed"
}

if (( USE_DOCKER )); then
  docker run --rm \
    --user "$(id -u):$(id -g)" \
    -v "$ROOT_DIR/..:/workspace/sdk_release" \
    -v "$V192_PCAP:/data/v192_demo.pcap:ro" \
    -v "$P60_IMU_PCAP:/data/p60_imu_demo.pcap:ro" \
    -v "$RESULT_DIR:/results" \
    "$IMAGE" bash -lc '
      V192_PCAP=/data/v192_demo.pcap \
      P60_IMU_PCAP=/data/p60_imu_demo.pcap \
      RESULT_DIR=/results \
      SAVE_FRAMES='"$SAVE_FRAMES"' \
      /workspace/sdk_release/lidar_driver_sdk/scripts/test_pcap.sh --no-docker
    '
else
  run_tests "$ROOT_DIR/.." "$V192_PCAP" "$P60_IMU_PCAP" "$RESULT_DIR"
fi
