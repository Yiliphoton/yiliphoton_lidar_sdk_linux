# Yiliphoton LiDAR Driver SDK

## Overview

**yiliphoton_lidar_sdk** is a pure C++ driver library developed by Yiliphoton for V192 / P60 LiDAR sensors. It has no ROS dependency and can be directly integrated into any Linux project.

```
  +---------------------------+
  |   yiliphoton_lidar_sdk    |
  |                           |
  |  +---------------------+  |
  |  |   Driver Core       |  |
  |  |  (decoder / input)  |  |
  |  +---------------------+  |
  |  |  Software Filter    |  |  ← prebuilt libaglidar_ag1.so
  |  +---------------------+  |
  +---------------------------+
```

### Supported LiDAR Models

| Model | Type |
|-------|------|
| V192  | 192 lines |
| P60   | 96 channels |

### Supported Point Types

| Type         | Fields |
|--------------|--------|
| `PointXYZI`  | x, y, z, intensity |
| `PointXYZIRT`| x, y, z, intensity, ring, range, confidence, timestamp |
| `PointXYZIRTC`| x, y, z, intensity, ring, range, confidence, timestamp, rgb |

---

## Directory Structure

```
yiliphoton_lidar_sdk/
├── src/ylp_driver/          # Driver headers (header-only + some .cpp)
│   ├── api/                 # Public interface: lidar_driver.hpp
│   ├── common/              # Logging, error codes, buffers, etc.
│   ├── driver/
│   │   ├── decoder/V192/    # V192 decoder
│   │   ├── decoder/P60/     # P60 decoder
│   │   ├── algorithm/       # Software filter interface
│   │   └── input/           # UDP / PCAP input
│   ├── config/              # Calibration files
│   └── msg/                 # Point cloud message definitions
├── lib/                     # Prebuilt software filter libraries
│   ├── libaglidar_ag1.so    # x86-64
│   └── arm64/libag1.so      # ARM64
├── cmake/                   # find_package support
├── demo/                    # Example programs
└── tool/                    # Utility programs
```

---

## Dependencies

```bash
sudo apt-get install -y libeigen3-dev libpcap-dev
```

---

## Build

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

### Common Build Options

| Option | Default | Description |
|--------|---------|-------------|
| `COMPILE_DEMOS` | ON | Build example programs |
| `COMPILE_TOOLS` | OFF | Build utility programs (requires PCL) |
| `ENABLE_TRANSFORM` | OFF | Enable coordinate transform (requires Eigen) |
| `DISABLE_PCAP_PARSE` | OFF | Disable PCAP parsing (no libpcap needed) |
| `ENABLE_PCL_POINTCLOUD` | OFF | Use PCL point cloud types |
| `ENABLE_POINT_EXTEND_FIELD` | OFF | Output azimuth/elevation extended fields |

---

## Integration

### Method 1: `find_package` (recommended, SDK installed to system or custom path)

Install the SDK first:

```bash
cd build
cmake --install . --prefix /usr/local   # or any path
```

Then in your project's `CMakeLists.txt`:

```cmake
find_package(ylp_driver REQUIRED)

target_link_libraries(your_target
  PRIVATE ylp_driver::ylp_driver_core)
```

If installed to a non-system path, specify `ylp_driver_DIR`:

```bash
cmake .. -Dylp_driver_DIR=/your/install/prefix/lib/cmake/ylp_driver
```

`pthread`, `Eigen3`, `pcap`, and `libaglidar_ag1.so` are handled automatically via transitive dependencies.

---

### Method 2: `add_subdirectory` (SDK as a source subdirectory)

```cmake
add_subdirectory(yiliphoton_lidar_sdk)

target_link_libraries(your_target
  PRIVATE ylp_driver::ylp_driver_core)
```

---

## Core API

Include a single header file:

```cpp
#include "ylp_driver/api/lidar_driver.hpp"
#include "ylp_driver/msg/point_cloud_msg.hpp"
```

### `LidarDriver<T_PointCloud>`

| Method | Description |
|--------|-------------|
| `regPointCloudCallback(cb_get, cb_put)` | Register point cloud memory management callbacks (see below) |
| `regExceptionCallback(cb)` | Register exception/status notification callback |
| `init(param)` | Initialize; returns `false` on failure |
| `start()` | Start receive/decode threads |
| `stop()` | Stop all threads |

### Memory Management

The driver uses a **dual-queue memory pool** to avoid frequent allocations. The user maintains two queues:

- **free_queue**: idle point cloud objects for the driver to fill
- **stuffed_queue**: filled point clouds for application processing

The driver calls `cb_get` to fetch an idle object, fills it, then calls `cb_put` to return it. The user processes and returns objects to `free_queue`.

```
Driver thread          User thread
     │  cb_get() ←─── free_queue ←── user pushes back after processing
     │
     │  (fill points)
     │
     │  cb_put() ──→ stuffed_queue ──→ user processes
```

> **Note**: `cb_get` and `cb_put` are called from the driver's decode thread and must not block. Perform point cloud processing in your own thread.

---

## Usage Examples

### 1. Online LiDAR (V192 / P60)

```cpp
#include "ylp_driver/api/lidar_driver.hpp"
#include "ylp_driver/msg/point_cloud_msg.hpp"
#include <thread>
#include <atomic>
#include <signal.h>

using namespace asensing::lidar;

// Choose point type: PointXYZI (coords + intensity) or PointXYZIRT (includes ring/timestamp)
using PointT    = PointXYZIRT;
using CloudMsg  = PointCloudT<PointT>;

SyncQueue<std::shared_ptr<CloudMsg>> free_queue;
SyncQueue<std::shared_ptr<CloudMsg>> stuffed_queue;
std::atomic<bool> running{true};

// Driver callback: get object from free queue (called in driver thread, must not block)
std::shared_ptr<CloudMsg> cbGet()
{
    auto msg = free_queue.pop();
    return msg ? msg : std::make_shared<CloudMsg>();
}

// Driver callback: push filled point cloud to processing queue (called in driver thread, must not block)
void cbPut(std::shared_ptr<CloudMsg> msg)
{
    stuffed_queue.push(msg);
}

// Exception/status callback
void cbException(const Error& err)
{
    // error_code_type: INFO_CODE / WARNING_CODE / ERROR_CODE
    YLP_WARNING << err.toString() << YLP_REND;
}

// Processing thread (can block)
void processThread()
{
    while (running)
    {
        auto msg = stuffed_queue.popWait();
        if (!msg) continue;

        // msg->points is std::vector<PointT>
        // msg->timestamp  frame timestamp (seconds, double)
        // msg->width      points per row
        // msg->height     number of rows
        // msg->is_dual    dual return mode
        for (const auto& pt : msg->points)
        {
            // pt.x, pt.y, pt.z  unit: meters
            // pt.intensity      reflectivity 0~255
            // pt.ring           channel index
            // pt.timestamp      point timestamp (seconds, double)
            (void)pt;
        }

        free_queue.push(msg);   // return to free queue after processing
    }
}

LidarDriver<CloudMsg> driver;

void sigHandler(int) { driver.stop(); running = false; }

int main()
{
    signal(SIGINT, sigHandler);

    YLPDriverParam param;
    param.lidar_type  = LidarType::V192;             // or LidarType::P60
    param.input_type  = InputType::ONLINE_LIDAR;
    param.input_param.msop_port  = 51180;             // LiDAR MSOP port
    param.input_param.difop_port = 9988;              // LiDAR DIFOP port
    // param.input_param.host_address = "192.168.1.100";  // Optional: bind to specific IP

    param.decoder_param.wait_for_difop  = false;      // true: wait for DIFOP before output
    param.decoder_param.use_lidar_clock = false;      // false: system clock for timestamps
    param.decoder_param.dense_points    = false;      // true: discard NaN points

    driver.regPointCloudCallback(cbGet, cbPut);
    driver.regExceptionCallback(cbException);

    if (!driver.init(param))
    {
        YLP_ERROR << "Initialization failed" << YLP_REND;
        return -1;
    }

    std::thread proc(processThread);
    driver.start();

    proc.join();
    return 0;
}
```

---

### 2. PCAP Replay

The only differences from online mode are the parameters and exit detection:

```cpp
#include "ylp_driver/api/lidar_driver.hpp"
#include "ylp_driver/msg/point_cloud_msg.hpp"
#include <thread>
#include <atomic>

using namespace asensing::lidar;
using PointT   = PointXYZI;
using CloudMsg = PointCloudT<PointT>;

SyncQueue<std::shared_ptr<CloudMsg>> free_queue;
SyncQueue<std::shared_ptr<CloudMsg>> stuffed_queue;
std::atomic<bool> running{true};

std::shared_ptr<CloudMsg> cbGet()
{
    auto msg = free_queue.pop();
    return msg ? msg : std::make_shared<CloudMsg>();
}

void cbPut(std::shared_ptr<CloudMsg> msg) { stuffed_queue.push(msg); }

void cbException(const Error& err)
{
    YLP_WARNING << err.toString() << YLP_REND;
    if (err.error_code == ERRCODE_PCAPEXIT)   // PCAP playback ended
        running = false;
}

void processThread()
{
    while (running)
    {
        auto msg = stuffed_queue.popWait();
        if (!msg) continue;

        YLP_MSG << "Frame " << msg->seq
                << "  points=" << msg->points.size()
                << "  timestamp=" << std::fixed << msg->timestamp
                << YLP_REND;

        free_queue.push(msg);
    }
}

int main(int argc, char* argv[])
{
    const char* pcap_path = (argc > 1) ? argv[1] : "your.pcap";

    YLPDriverParam param;
    param.lidar_type  = LidarType::P60;              // set based on recorded LiDAR model
    param.input_type  = InputType::PCAP_FILE;
    param.input_param.pcap_path   = pcap_path;
    param.input_param.pcap_repeat = false;            // true: loop playback
    param.input_param.pcap_rate   = 1.0f;             // playback speed multiplier
    param.input_param.msop_port   = 51180;            // must match recorded port
    param.input_param.difop_port  = 9988;

    param.decoder_param.wait_for_difop  = false;
    param.decoder_param.use_lidar_clock = false;

    LidarDriver<CloudMsg> driver;
    driver.regPointCloudCallback(cbGet, cbPut);
    driver.regExceptionCallback(cbException);

    if (!driver.init(param)) return -1;

    std::thread proc(processThread);
    driver.start();

    while (running)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    driver.stop();
    proc.join();
    return 0;
}
```

---

### 3. Multiple LiDARs

Instantiate one `LidarDriver` per LiDAR with different ports:

```cpp
YLPDriverParam param_left;
param_left.lidar_type             = LidarType::V192;
param_left.input_type             = InputType::ONLINE_LIDAR;
param_left.input_param.msop_port  = 51180;
param_left.input_param.difop_port = 9988;

YLPDriverParam param_right;
param_right.lidar_type             = LidarType::V192;
param_right.input_type             = InputType::ONLINE_LIDAR;
param_right.input_param.msop_port  = 51181;   // different port
param_right.input_param.difop_port = 9989;

LidarDriver<CloudMsg> driver_left, driver_right;
// register callbacks, init, start for each...
```

---

## Key Parameters

### `YLPDecoderParam`

| Parameter | Default | Description |
|-----------|---------|-------------|
| `wait_for_difop` | true | Wait for DIFOP packet before output; recommend false for PCAP |
| `use_lidar_clock` | false | false: system clock; true: LiDAR internal clock |
| `dense_points` | false | true: discard NaN points (distance=0); false: keep |
| `enable_dual_mode` | true | Dual-return mode |
| `calib_file` | `""` | Calibration filename or absolute path; auto-detected when empty |
| `min_distance` / `max_distance` | 0 | Distance filter (meters); disabled when both are 0 |

### `YLPInputParam`

| Parameter | Default | Description |
|-----------|---------|-------------|
| `msop_port` | 51180 | LiDAR data packet port |
| `difop_port` | 9988 | LiDAR config packet port |
| `host_address` | `"0.0.0.0"` | Local IP to bind; specify for multi-NIC setups |
| `pcap_path` | `""` | Absolute path to PCAP file |
| `pcap_repeat` | true | Loop playback after reaching end |
| `pcap_rate` | 1.0 | Playback speed (2.0 = 2× speed) |

---

## Error Codes

| Error Code | Severity | Meaning |
|------------|----------|---------|
| `ERRCODE_PCAPEXIT` | INFO | PCAP playback ended (when repeat is off) |
| `ERRCODE_PCAPREPEAT` | INFO | PCAP reached end and restarted |
| `ERRCODE_MSOPTIMEOUT` | WARNING | No data packets received for 1 second |
| `ERRCODE_ZEROPOINTS` | WARNING | Zero points in current frame |
| `ERRCODE_PKTBUFOVERFLOW` | WARNING | Packet queue overflow (processing too slow) |
| `ERRCODE_PCAPWRONGPATH` | ERROR | PCAP file path not found |
| `ERRCODE_STARTBEFOREINIT` | ERROR | `start()` called before `init()` |

---

## Calibration Files

Calibration files are located at `src/ylp_driver/config/`, and after installation at `<prefix>/share/ylp_driver/config/`.

The driver searches for calibration files in this order:
1. Absolute path specified by `calib_file`
2. File with the same name in the current working directory
3. File with the same name next to the executable (auto-detected, **usually no manual config needed**)

To manually specify:

```cpp
param.decoder_param.calib_file = "/usr/local/share/ylp_driver/config/V192-Correction.json";
```

---

## FAQ

**Q: No point cloud received?**  
Check that the firewall allows MSOP port (default 51180) and DIFOP port (default 9988):
```bash
sudo ufw allow 51180/udp
sudo ufw allow 9988/udp
```

**Q: `libaglidar_ag1.so` not found at runtime?**  
```bash
export LD_LIBRARY_PATH=/path/to/yiliphoton_lidar_sdk/lib:$LD_LIBRARY_PATH
# or after installation
sudo ldconfig
```

**Q: How to use on ARM platforms?**  
Modify the library search path in CMakeLists.txt to point to `lib/arm64`:
```cmake
find_library(YLP_AG1_LIB aglidar_ag1 PATHS ${CMAKE_CURRENT_LIST_DIR}/lib/arm64)
```
