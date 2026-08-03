**中文** | [English](README.md)

# Yiliphoton LiDAR Driver SDK

## 概述

**yiliphoton_lidar_sdk** 是一力光子为 V192 / P60 激光雷达配套的纯 C++ 驱动库，不依赖 ROS，可直接集成到任意 Linux 工程中。

```
  +---------------------------+
  |   yiliphoton_lidar_sdk    |
  |                           |
  |  +---------------------+  |
  |  |   Driver Core       |  |
  |  |  (decoder / input)  |  |
  |  +---------------------+  |
  |  |  Software Filter    |  |  ← 预编译 libaglidar_ag1.so
  |  +---------------------+  |
  +---------------------------+
```

### 支持的雷达型号

| 型号 | 类型 |
|------|------|
| V192 | 192 线 |
| P60  | 96 通道 |

### 支持的点类型

| 类型 | 字段 |
|------|------|
| `PointXYZI`   | x, y, z, intensity |
| `PointXYZIRT` | x, y, z, intensity, ring, range, confidence, timestamp |
| `PointXYZIRTC`| x, y, z, intensity, ring, range, confidence, timestamp, rgb |

---

## 目录结构

```
yiliphoton_lidar_sdk/
├── src/ylp_driver/          # 驱动头文件（header-only + 少量 .cpp）
│   ├── api/                 # 对外接口 lidar_driver.hpp
│   ├── common/              # 日志、错误码、缓冲区等
│   ├── driver/
│   │   ├── decoder/V192/    # V192 解码器
│   │   ├── decoder/P60/     # P60 解码器
│   │   ├── algorithm/       # 软件滤波接口
│   │   └── input/           # UDP / PCAP 输入
│   ├── config/              # 标定文件
│   └── msg/                 # 点云消息定义
├── lib/                     # 预编译软件滤波库
│   ├── libaglidar_ag1.so    # x86-64
│   └── arm64/libag1.so      # ARM64
├── cmake/                   # find_package 支持
├── demo/                    # 示例程序
└── tool/                    # 工具程序
```

---

## 安装依赖

```bash
sudo apt-get install -y libeigen3-dev libpcap-dev
```

---

## 编译

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

### 常用编译选项

| 选项 | 默认 | 说明 |
|------|------|------|
| `COMPILE_DEMOS` | ON | 编译示例程序 |
| `COMPILE_TOOLS` | OFF | 编译工具程序（需要 PCL） |
| `ENABLE_TRANSFORM` | OFF | 启用坐标变换（需要 Eigen） |
| `DISABLE_PCAP_PARSE` | OFF | 禁用 PCAP 解析（不需要 libpcap） |
| `ENABLE_PCL_POINTCLOUD` | OFF | 使用 PCL 点云类型 |
| `ENABLE_POINT_EXTEND_FIELD` | OFF | 输出方位角/俯仰角扩展字段 |

---

## 集成到自己的工程

### 方式一：`find_package`（推荐，SDK 已安装到系统或指定路径）

先安装 SDK：

```bash
cd build
cmake --install . --prefix /usr/local   # 或任意路径
```

然后在自己项目的 `CMakeLists.txt` 中：

```cmake
find_package(ylp_driver REQUIRED)

target_link_libraries(your_target
  PRIVATE ylp_driver::ylp_driver_core)
```

如果安装到非系统路径，指定 `ylp_driver_DIR`：

```bash
cmake .. -Dylp_driver_DIR=/your/install/prefix/lib/cmake/ylp_driver
```

`pthread`、`Eigen3`、`pcap`、`libaglidar_ag1.so` 均通过传递依赖自动处理，不需要手动添加。

---

### 方式二：`add_subdirectory`（SDK 作为源码子目录）

```cmake
add_subdirectory(yiliphoton_lidar_sdk)

target_link_libraries(your_target
  PRIVATE ylp_driver::ylp_driver_core)
```

---

## 核心 API

只需包含一个头文件：

```cpp
#include "ylp_driver/api/lidar_driver.hpp"
#include "ylp_driver/msg/point_cloud_msg.hpp"
```

### `LidarDriver<T_PointCloud>`

| 方法 | 说明 |
|------|------|
| `regPointCloudCallback(cb_get, cb_put)` | 注册点云内存管理回调（见下文） |
| `regExceptionCallback(cb)` | 注册异常/状态通知回调 |
| `init(param)` | 初始化，返回 `false` 表示失败 |
| `start()` | 启动接收/解码线程 |
| `stop()` | 停止所有线程 |

### 内存管理机制

驱动使用**双队列内存池**避免频繁分配。用户维护两个队列：

- **free_queue**：空闲的点云对象，供驱动填充
- **stuffed_queue**：已填充的点云，供业务处理

驱动调用 `cb_get` 取一个空闲对象，填满后调用 `cb_put` 交还。用户处理完后把对象放回 `free_queue`。

```
Driver thread          User thread
     │  cb_get() ←─── free_queue ←── user pushes back after processing
     │
     │  (fill points)
     │
     │  cb_put() ──→ stuffed_queue ──→ user processes
```

> **注意**：`cb_get` 和 `cb_put` 在驱动的解码线程中被调用，不能做耗时操作。点云处理放在用户自己的线程里。

---

## 使用示例

### 1. 在线雷达（V192 / P60）

```cpp
#include "ylp_driver/api/lidar_driver.hpp"
#include "ylp_driver/msg/point_cloud_msg.hpp"
#include <thread>
#include <atomic>
#include <signal.h>

using namespace asensing::lidar;

// 选择点类型：PointXYZI（仅坐标+强度）或 PointXYZIRT（含 ring/timestamp）
using PointT    = PointXYZIRT;
using CloudMsg  = PointCloudT<PointT>;

SyncQueue<std::shared_ptr<CloudMsg>> free_queue;
SyncQueue<std::shared_ptr<CloudMsg>> stuffed_queue;
std::atomic<bool> running{true};

// 驱动回调：从空闲队列取对象（在驱动线程中调用，不要耗时）
std::shared_ptr<CloudMsg> cbGet()
{
    auto msg = free_queue.pop();
    return msg ? msg : std::make_shared<CloudMsg>();
}

// 驱动回调：把填好的点云投入处理队列（在驱动线程中调用，不要耗时）
void cbPut(std::shared_ptr<CloudMsg> msg)
{
    stuffed_queue.push(msg);
}

// 异常/状态回调
void cbException(const Error& err)
{
    // error_code_type: INFO_CODE / WARNING_CODE / ERROR_CODE
    YLP_WARNING << err.toString() << YLP_REND;
}

// 业务处理线程（可以耗时）
void processThread()
{
    while (running)
    {
        auto msg = stuffed_queue.popWait();
        if (!msg) continue;

        // msg->points 是 std::vector<PointT>
        // msg->timestamp  帧时间戳（秒，double）
        // msg->width      每行点数
        // msg->height     行数
        // msg->is_dual    是否双回波
        for (const auto& pt : msg->points)
        {
            // pt.x, pt.y, pt.z  单位：米
            // pt.intensity       反射强度 0~255
            // pt.ring            通道编号
            // pt.timestamp       点时间戳（秒，double）
            (void)pt;
        }

        free_queue.push(msg);   // 用完放回空闲队列
    }
}

LidarDriver<CloudMsg> driver;

void sigHandler(int) { driver.stop(); running = false; }

int main()
{
    signal(SIGINT, sigHandler);

    YLPDriverParam param;
    param.lidar_type  = LidarType::V192;          // 或 LidarType::P60
    param.input_type  = InputType::ONLINE_LIDAR;
    param.input_param.msop_port  = 51180;          // 雷达 MSOP 端口
    param.input_param.difop_port = 9988;           // 雷达 DIFOP 端口
    // param.input_param.host_address = "192.168.1.100";  // 可选：绑定本机 IP

    param.decoder_param.wait_for_difop  = false;   // true：等到 DIFOP 包才出点
    param.decoder_param.use_lidar_clock = false;   // false：用系统时钟打时间戳
    param.decoder_param.dense_points    = false;   // true：丢弃 NaN 点

    driver.regPointCloudCallback(cbGet, cbPut);
    driver.regExceptionCallback(cbException);

    if (!driver.init(param))
    {
        YLP_ERROR << "初始化失败" << YLP_REND;
        return -1;
    }

    std::thread proc(processThread);
    driver.start();

    proc.join();
    return 0;
}
```

---

### 2. 回放 PCAP 文件

与在线雷达的区别只有参数和退出检测：

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
    if (err.error_code == ERRCODE_PCAPEXIT)   // PCAP 播放结束
        running = false;
}

void processThread()
{
    while (running)
    {
        auto msg = stuffed_queue.popWait();
        if (!msg) continue;

        YLP_MSG << "帧 " << msg->seq
                << "  点数=" << msg->points.size()
                << "  时间戳=" << std::fixed << msg->timestamp
                << YLP_REND;

        free_queue.push(msg);
    }
}

int main(int argc, char* argv[])
{
    const char* pcap_path = (argc > 1) ? argv[1] : "your.pcap";

    YLPDriverParam param;
    param.lidar_type  = LidarType::P60;           // 根据录制雷达选择
    param.input_type  = InputType::PCAP_FILE;
    param.input_param.pcap_path   = pcap_path;
    param.input_param.pcap_repeat = false;         // true：循环播放
    param.input_param.pcap_rate   = 1.0f;          // 播放速率倍数
    param.input_param.msop_port   = 51180;         // 必须与录制时端口一致
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

### 3. 多雷达同时接收

每个雷达实例化一个 `LidarDriver`，端口不同即可：

```cpp
YLPDriverParam param_left;
param_left.lidar_type             = LidarType::V192;
param_left.input_type             = InputType::ONLINE_LIDAR;
param_left.input_param.msop_port  = 51180;
param_left.input_param.difop_port = 9988;

YLPDriverParam param_right;
param_right.lidar_type             = LidarType::V192;
param_right.input_type             = InputType::ONLINE_LIDAR;
param_right.input_param.msop_port  = 51181;   // 不同端口
param_right.input_param.difop_port = 9989;

LidarDriver<CloudMsg> driver_left, driver_right;
// 分别注册回调、init、start ...
```

---

## 关键参数说明

### `YLPDecoderParam`

| 参数 | 默认 | 说明 |
|------|------|------|
| `wait_for_difop` | true | 等收到 DIFOP 包才开始出点；PCAP 回放建议设 false |
| `use_lidar_clock` | false | false：用系统时钟；true：用雷达内部时钟 |
| `dense_points` | false | true：丢弃 NaN 点（距离为 0）；false：保留 |
| `enable_dual_mode` | true | 双回波模式 |
| `calib_file` | `""` | 标定文件名或绝对路径；为空时自动用默认文件名 |
| `min_distance` / `max_distance` | 0 | 距离过滤（米），两者均为 0 时不过滤 |

### `YLPInputParam`

| 参数 | 默认 | 说明 |
|------|------|------|
| `msop_port` | 51180 | 雷达数据包端口 |
| `difop_port` | 9988 | 雷达配置包端口 |
| `host_address` | `"0.0.0.0"` | 绑定本机 IP，多网卡时需指定 |
| `pcap_path` | `""` | PCAP 文件绝对路径 |
| `pcap_repeat` | true | PCAP 到末尾是否循环 |
| `pcap_rate` | 1.0 | 播放速率（2.0 = 2 倍速） |

---

## 错误码

| 错误码 | 类型 | 含义 |
|--------|------|------|
| `ERRCODE_PCAPEXIT` | INFO | PCAP 播放结束（不重复时触发） |
| `ERRCODE_PCAPREPEAT` | INFO | PCAP 到末尾并重新开始 |
| `ERRCODE_MSOPTIMEOUT` | WARNING | 超过 1 秒未收到数据包 |
| `ERRCODE_ZEROPOINTS` | WARNING | 本帧点数为 0 |
| `ERRCODE_PKTBUFOVERFLOW` | WARNING | 包队列溢出（处理速度跟不上） |
| `ERRCODE_PCAPWRONGPATH` | ERROR | PCAP 文件路径不存在 |
| `ERRCODE_STARTBEFOREINIT` | ERROR | 未调用 `init()` 就调用了 `start()` |

---

## 标定文件

标定文件位于 `src/ylp_driver/config/`，安装后在 `<prefix>/share/ylp_driver/config/`。

驱动按以下顺序查找标定文件：
1. `calib_file` 指定的绝对路径
2. 当前工作目录下同名文件
3. 可执行文件同级目录下同名文件（自动查找，**通常不需要手动配置**）

手动指定路径：

```cpp
param.decoder_param.calib_file = "/usr/local/share/ylp_driver/config/V192-Correction.json";
```

---

## 常见问题

**Q: 收不到点云？**  
检查防火墙是否放行 MSOP 端口（默认 51180）和 DIFOP 端口（默认 9988）：
```bash
sudo ufw allow 51180/udp
sudo ufw allow 9988/udp
```

**Q: 运行时找不到 `libaglidar_ag1.so`？**  
```bash
export LD_LIBRARY_PATH=/path/to/yiliphoton_lidar_sdk/lib:$LD_LIBRARY_PATH
# 或安装后直接 ldconfig
sudo ldconfig
```

**Q: ARM 平台如何使用？**  
在 CMakeLists.txt 中修改库搜索路径指向 `lib/arm64`：
```cmake
find_library(YLP_AG1_LIB aglidar_ag1 PATHS ${CMAKE_CURRENT_LIST_DIR}/lib/arm64)
```

