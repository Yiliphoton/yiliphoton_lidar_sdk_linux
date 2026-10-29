#!/usr/bin/env bash

set -euo pipefail

export LC_ALL=C

package_version=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --version)
      package_version="${2:-}"
      shift 2
      ;;
    --version=*)
      package_version="${1#*=}"
      shift
      ;;
    -h|--help)
      cat <<'EOF'
Usage: package.sh [--version VERSION]

Build and package the LiDAR SDK.
EOF
      exit 0
      ;;
    *)
      printf 'Unknown argument: %s
' "$1" >&2
      exit 1
      ;;
  esac
done

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="$root_dir/out/build/release"
dist_dir="$root_dir/out/dist"
release_dir="$root_dir/../release_packages"
mkdir -p "$root_dir/out" "$dist_dir" "$release_dir"
stage_dir="$(mktemp -d "$root_dir/out/stage.XXXXXX")"

cleanup() {
  rm -rf "$stage_dir"
}
trap cleanup EXIT

cd "$root_dir"

cmake_args=(
  -S "$root_dir"
  -B "$build_dir"
  -DCMAKE_BUILD_TYPE=Release
  -DBUILD_DEMO=ON
)
if [[ -n "$package_version" ]]; then
  cmake_args+=("-DLIDAR_SDK_VERSION=$package_version")
fi

cmake "${cmake_args[@]}"
cmake --build "$build_dir" --parallel

package_file_name="$(sed -n 's/^set(CPACK_PACKAGE_FILE_NAME "\(.*\)")$/\1/p' "$build_dir/CPackConfig.cmake" | head -n 1)"
package_version="$(sed -n 's/^set(CPACK_PACKAGE_VERSION "\(.*\)")$/\1/p' "$build_dir/CPackConfig.cmake" | head -n 1)"
package_arch="${package_file_name##*-}"
sdk_stage="$stage_dir/sdk"

if [[ -z "$package_file_name" || -z "$package_version" ]]; then
  printf 'Failed to read package metadata from CPackConfig.cmake.\n' >&2
  exit 1
fi

case "$package_arch" in
  x86_64)
    expected_file_arch='x86-64'
    ;;
  aarch64)
    expected_file_arch='ARM aarch64'
    ;;
  *)
    printf 'Unsupported package architecture: %s\n' "$package_arch" >&2
    exit 1
    ;;
esac

cmake --install "$build_dir" --prefix "$sdk_stage"

sdk_library="$sdk_stage/lib/liblidar_sdk.so"
if [[ ! -f "$sdk_library" || ! -f "$sdk_stage/lib/liblidar_algo.so" ]]; then
  printf 'SDK or algorithm shared library is missing from the install tree.\n' >&2
  exit 1
fi

if ! file -L "$sdk_library" | grep -Fq "$expected_file_arch"; then
  printf 'SDK library architecture does not match package architecture %s.\n' "$package_arch" >&2
  file -L "$sdk_library" >&2
  exit 1
fi

for symbol in \
  lidar_sdk_config_init \
  lidar_sdk_version \
  lidar_driver_create \
  lidar_driver_destroy \
  lidar_driver_set_frame_callback \
  lidar_driver_set_error_callback \
  lidar_driver_set_imu_callback \
  lidar_driver_init \
  lidar_driver_start \
  lidar_driver_stop; do
  if ! nm -D --defined-only "$sdk_library" | awk '{print $3}' | grep -Fxq "$symbol"; then
    printf 'Public API symbol is missing from the SDK library: %s\n' "$symbol" >&2
    exit 1
  fi
done

if ! readelf -d "$sdk_library" | grep -Fq 'Shared library: [liblidar_algo.so]'; then
  printf 'SDK library does not declare the required liblidar_algo.so dependency.\n' >&2
  exit 1
fi

if ! readelf -d "$sdk_library" | grep -Eq '(Library runpath|Library rpath): \[.*\$ORIGIN.*\]'; then
  printf 'SDK library does not use the required $ORIGIN runtime search path.\n' >&2
  exit 1
fi

for required_file in \
  "$sdk_stage/include/lidar_sdk/lidar_sdk.h" \
  "$sdk_stage/share/lidar_sdk/config/P60-Correction.json" \
  "$sdk_stage/share/lidar_sdk/config/V192-Correction.json" \
  "$sdk_stage/lib/cmake/lidar_sdk/lidar_sdkConfig.cmake" \
  "$sdk_stage/bin/demo" \
  "$sdk_stage/README.md" \
  "$sdk_stage/QUICKSTART.md" \
  "$sdk_stage/share/doc/lidar_sdk/BUILD-INFO.txt" \
  "$sdk_stage/share/lidar_sdk/examples/consumer_cmake/CMakeLists.txt" \
  "$sdk_stage/share/lidar_sdk/examples/consumer_cmake/main.c"; do
  if [[ ! -f "$required_file" ]]; then
    printf 'Required delivery file is missing: %s\n' "$required_file" >&2
    exit 1
  fi
done

consumer_build="$stage_dir/consumer-build"
cmake -S "$root_dir/examples/consumer_cmake" -B "$consumer_build" \
  -DCMAKE_PREFIX_PATH="$sdk_stage"
cmake --build "$consumer_build" --parallel
LD_LIBRARY_PATH="$sdk_stage/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
  "$consumer_build/lidar_sdk_consumer"

cpack --config "$build_dir/CPackConfig.cmake" -B "$dist_dir"

binary_archive="$dist_dir/${package_file_name}.tar.gz"
if [[ ! -f "$binary_archive" ]]; then
  printf 'Failed to create binary release archive.\n' >&2
  exit 1
fi

source_bundle_name="lidar_sdk-${package_version}-source"
source_root="$stage_dir/$source_bundle_name"
mkdir -p "$source_root"

tar \
  --exclude='./.git' \
  --exclude='./build' \
  --exclude='./out' \
  --exclude='./package' \
  -C "$root_dir" -cf - . | tar -C "$source_root" -xf -

cat > "$source_root/DELIVERY.md" <<'EOF'
# LiDAR SDK Source Delivery / 源码包说明

- Start with `QUICKSTART.md` (Chinese) or `QUICKSTART_EN.md` (English).
- 请先阅读 `QUICKSTART.md`，按步骤构建 SDK、运行 demo 并验证客户工程。
- This archive is the complete SDK source tree. Build it on the target Linux architecture.
- The source tree includes platform-specific private algorithm libraries under `lib/`.
- Native x86-64 builds use `lib/x86-64/liblidar_algo.so`; native ARM64 builds use `lib/arm64/liblidar_algo.so` automatically.
- Prebuilt binary SDK archives, when provided, are architecture-specific and must only be used on the matching target architecture.
EOF

source_archive="$dist_dir/${source_bundle_name}.tar.gz"
tar -C "$stage_dir" -czf "$source_archive" "$source_bundle_name"

source_contents="$stage_dir/source-archive-contents.txt"
tar -tzf "$source_archive" > "$source_contents"

for required_source_file in \
  CMakeLists.txt \
  README.md \
  README_EN.md \
  QUICKSTART.md \
  QUICKSTART_EN.md \
  QUICKSTART_BINARY.md \
  QUICKSTART_BINARY_EN.md \
  DELIVERY.md \
  include/lidar_sdk/lidar_sdk.h \
  lib/x86-64/liblidar_algo.so \
  lib/arm64/liblidar_algo.so \
  resources/calibration/P60-Correction.json \
  resources/calibration/V192-Correction.json \
  examples/consumer_cmake/CMakeLists.txt \
  examples/consumer_cmake/main.c; do
  if ! grep -Fxq "$source_bundle_name/$required_source_file" "$source_contents"; then
    printf 'Required source delivery file is missing: %s\n' "$required_source_file" >&2
    exit 1
  fi
done

for forbidden_source_directory in build out package sdk; do
  if grep -Fq "$source_bundle_name/$forbidden_source_directory/" "$source_contents"; then
    printf 'Source archive contains a generated top-level directory: %s\n' \
      "$forbidden_source_directory" >&2
    exit 1
  fi
done

legacy_source_archive="$dist_dir/${package_file_name}-source.tar.gz"
rm -f "$legacy_source_archive"

checksum_file="$dist_dir/lidar_sdk-${package_version}-SHA256SUMS.txt"
(
  cd "$dist_dir"
  sha256sum "$(basename "$binary_archive")" "$(basename "$source_archive")"
) > "$checksum_file"

release_binary_archive="$release_dir/$(basename "$binary_archive")"
release_source_archive="$release_dir/$(basename "$source_archive")"
release_checksum_file="$release_dir/$(basename "$checksum_file")"
cp -f "$binary_archive" "$release_binary_archive"
cp -f "$source_archive" "$release_source_archive"
cp -f "$checksum_file" "$release_checksum_file"

printf 'Binary release archive: %s\n' "$binary_archive"
printf 'Source release archive: %s\n' "$source_archive"
printf 'Checksums: %s\n' "$checksum_file"
printf 'Release packages copied to: %s\n' "$release_dir"
