# SDK Versioning and Release Guide

This guide is for SDK maintainers. It describes the version definition, one-command packaging process, delivery artifacts, and cleanup rules.

## 1. Version Definition

Define the SDK release version only once at the top of `CMakeLists.txt`:

```cmake
project(lidar_sdk VERSION 1.1.0 LANGUAGES C CXX)
```

For a new release, replace `1.1.0` with the target version, for example:

```cmake
project(lidar_sdk VERSION 1.1.1 LANGUAGES C CXX)
```

After running the release script, this version is applied automatically to the CPack archive names and the real `liblidar_sdk.so.<full-version>` file.

### ABI Major Version

The shared-library ABI major version is controlled by:

```cmake
SOVERSION 1
```

- Keep `SOVERSION 1` for bug fixes, compatible features, and changes that preserve binary compatibility of the public C API.
- Increase it to `SOVERSION 2` when changing or removing public function signatures, changing public structure layout, or otherwise breaking ABI compatibility. Also increase the release version.

Do not change `SOVERSION` for ordinary feature releases.

## 2. One-command Packaging

Run the following on the target Linux architecture:

```bash
cd /home/zyy/sdk_release/lidar_driver_sdk
./scripts/package.sh
```

The script configures and builds the Release SDK for the current Linux architecture, installs it to a temporary staging directory, validates the library architecture, public API symbols, algorithm dependency, `$ORIGIN` runtime path, calibration JSON files, demo, and CMake package files, then builds and runs an independent `find_package(lidar_sdk)` client example. It creates the binary CPack archive only after those checks pass, then creates a standalone source archive and SHA-256 checksums.

The script exits on any validation failure. Do not deliver archives from a failed release attempt.

## 3. Release Outputs

The release artifacts are written to:

```text
out/dist/
```

For version `1.1.0` on x86-64, the artifacts are:

```text
lidar_sdk-1.1.0-linux-x86_64.tar.gz
lidar_sdk-1.1.0-source.tar.gz
lidar_sdk-1.1.0-SHA256SUMS.txt
```

### Standard Binary Package

`lidar_sdk-<version>-linux-<architecture>.tar.gz` is the normal customer delivery. It includes the public header, SDK and algorithm libraries, calibration JSON files, CMake package files, demo, README files, and `BUILD-INFO.txt`. Use it only on the architecture and compatible runtime named by the archive. Customers normally use this package without rebuilding the SDK.

### Standalone Source Package

`lidar_sdk-<version>-source.tar.gz` is the build baseline for every platform. It contains the complete project root:

```text
include/, src/, resources/, lib/, cmake/, demo/, examples/, scripts/
```

The source package includes private algorithm libraries for both `lib/x86-64/` and `lib/arm64/`. Build it natively on the target device; CMake selects the matching algorithm library automatically. It intentionally does not contain any prebuilt `liblidar_sdk.so`, so an ARM64 user cannot accidentally use an x86-64 binary.

Provide a binary archive for a platform only after it has been built and validated on that architecture. Provide the source package for special systems, unqualified binary architectures, debugging, or SDK customization.

## 4. Cleanup Rules

```text
build/                 # Legacy manual build cache; safe to delete
out/build/             # One-command build cache; regenerated automatically
out/dist/              # Final delivery archives; retain or clean old releases manually
package/               # Legacy manual install directory; not used by the current flow
```

Do not deliver files copied manually from `build/` or `package/`. Always run `./scripts/package.sh` and deliver the archives from `out/dist/`.

## 5. Architecture and Runtime Compatibility

- Build and validate x86-64 and ARM64 binary releases separately on their respective Linux architectures.
- For an unvalidated architecture, provide only the source package and do not claim a prebuilt binary SDK is available.
- The target system's glibc and `libstdc++` must be compatible with the build environment. Build on an appropriately older compatible system if the target runtime is older.
- Keep `liblidar_sdk.so` and `liblidar_algo.so` in the same dynamic-library search path. Applications may place calibration JSON files in any directory by setting `calibration_path`.

## 6. Pre-release Checklist

1. Update `VERSION` in `CMakeLists.txt`.
2. Run `./scripts/package.sh` successfully.
3. Confirm `out/dist/` contains the current-architecture binary archive, standalone source archive, and SHA-256 checksum file.
4. Confirm `BUILD-INFO.txt`, archive names, and actual library architecture agree.
5. Do not deliver stale archives whose names may not match their contents.
6. Run the demo or a customer-style application on the target architecture or an equivalent environment.
