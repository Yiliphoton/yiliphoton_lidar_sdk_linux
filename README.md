# LiDAR SDK

本 SDK 支持 P60、P60_IMU 与 V192 激光雷达。提供二进制包和源码包；SDK 负责 UDP 接收、协议解码、点云滤波、内部队列和工作线程。

## 二进制交付包

```text
include/lidar_sdk/lidar_sdk.h
lib/liblidar_sdk.so
lib/liblidar_algo.so
share/lidar_sdk/config/
lib/cmake/lidar_sdk/
```

二进制包只能在包名标注的目标架构上使用，例如 `linux-x86_64` 不能用于 ARM64。`liblidar_sdk.so` 与 `liblidar_algo.so` 必须位于同一动态库搜索路径；放在同一个 `lib/` 目录是推荐方式。

`share/lidar_sdk/config/` 是未设置标定文件路径时的默认位置。应用设置 `calibration_path` 后，可将标定 JSON 放在任意配置目录。二进制包运行时还依赖目标系统提供 `libpcap`、`libstdc++` 等基础运行库。

## 源码包与原生构建

```text
include/lidar_sdk/       # 对外 ABI：唯一需要交付给应用的头文件
src/api/                    # C ABI 适配层、配置转换、回调派发
src/core/                   # 私有引擎：UDP、协议解码、滤波、队列和线程
resources/calibration/      # 随 SDK 安装的 P60/P60_IMU/V192 标定文件
demo/                       # 仅演示公开 API 的 C 示例
lib/                        # 按平台提供的私有算法动态库
cmake/                      # find_package 配置文件
```

源码包是完整项目根目录，不包含任何其他架构的预编译 `sdk/` 子目录。请在目标设备进入解压后的源码根目录构建。原生 x86-64 构建自动使用 `lib/x86-64/liblidar_algo.so`；原生 ARM64 构建自动使用 `lib/arm64/liblidar_algo.so`。不支持的架构会在 CMake 配置阶段直接报错。

源码包用户请先阅读 [QUICKSTART.md](QUICKSTART.md)。

## 编译与安装

从源码构建 SDK 的依赖：C/C++ 编译器、CMake、`libeigen3-dev`、`libpcap-dev`。使用已安装二进制 SDK 的应用不需要这些构建依赖。Ubuntu/Debian 示例：

```bash
sudo apt update
sudo apt install -y build-essential cmake libeigen3-dev libpcap-dev
```

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
cmake --install build --prefix /opt/lidar_sdk
```

源码包中的 `examples/consumer_cmake/` 是独立客户工程示例，可用于验证安装后的 `find_package(lidar_sdk)` 集成。

## 应用集成

应用 CMakeLists.txt：

```cmake
cmake_minimum_required(VERSION 3.16)
project(lidar_application LANGUAGES C CXX)

find_package(lidar_sdk CONFIG REQUIRED)

add_executable(lidar_application main.c)
target_link_libraries(lidar_application PRIVATE lidar_sdk::lidar_sdk)
```

假设 SDK 安装到 `/opt/lidar_sdk`，配置应用：

```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH=/opt/lidar_sdk
cmake --build build -j
```

运行时请确保 `liblidar_sdk.so` 与 `liblidar_algo.so` 位于同一动态库搜索路径。若应用本身未设置 RPATH，可执行：

```bash
export LD_LIBRARY_PATH=/opt/lidar_sdk/lib:${LD_LIBRARY_PATH}
./build/lidar_application
```

```c
#include <lidar_sdk/lidar_sdk.h>

#include <stddef.h>
#include <string.h>

static void on_frame(const lidar_sdk_frame_t* frame, void* user_data) {
  (void)user_data;
  /* 在此将 frame->points 转换为业务、PCL 或 ROS 点云类型。 */
}

static void on_error(int32_t code, const char* message, void* user_data) {
  (void)code;
  (void)message;
  (void)user_data;
}

lidar_sdk_config_t config;
lidar_sdk_config_init(&config);
config.model = LIDAR_SDK_MODEL_P60_IMU;
config.msop_port = 8899;

lidar_driver_t* driver = lidar_driver_create();
if (driver == NULL) {
  return 1;
}
lidar_driver_set_frame_callback(driver, on_frame, NULL);
lidar_driver_set_error_callback(driver, on_error, NULL);

if (lidar_driver_init(driver, &config) != LIDAR_SDK_SUCCESS ||
    lidar_driver_start(driver) != LIDAR_SDK_SUCCESS) {
  lidar_driver_destroy(driver);
  return 1;
}

/* ... */

lidar_driver_stop(driver);
lidar_driver_destroy(driver);
```

## 配置说明

始终先调用 `lidar_sdk_config_init()`，再覆盖需要的字段。当前 P60、P60_IMU、V192 的现场 MSOP 示例端口均为 `8899`，默认值也为 `8899`；仍建议应用显式设置端口，避免依赖雷达历史配置。

| 字段 | 含义与建议值 |
| --- | --- |
| `model` | `LIDAR_SDK_MODEL_P60`、`LIDAR_SDK_MODEL_P60_IMU` 或 `LIDAR_SDK_MODEL_V192`。 |
| `input_type` | 在线雷达使用 `LIDAR_SDK_INPUT_ONLINE`；离线文件使用 `LIDAR_SDK_INPUT_PCAP`。 |
| `msop_port` | 本机接收 MSOP 的 UDP 端口；当前 P60/P60_IMU/V192 示例为 `8899`。 |
| `difop_port` | P60/P60_IMU/V192 当前不使用 DIFOP；保持 `0`，避免额外绑定 UDP 端口。 |
| `host_address` | 本机绑定 IP。广播场景设为 `0.0.0.0`；单播场景填写本机接收网卡 IP，例如 `192.168.1.7`。不要填雷达自身 IP。 |
| `group_address` | 仅组播场景填写组播地址；单播和广播保持 `0.0.0.0`。 |
| `calibration_path` | 为空时自动使用安装目录内置标定 JSON；可填写绝对路径覆盖。 |
| `min_distance_m` / `max_distance_m` | 距离裁剪范围，单位米；默认范围 `0.0~300.0`。一般无需设置；如需自定义，`max_distance_m` 必须大于 `min_distance_m`。 |
| `pcap_path` | PCAP 文件绝对路径，仅 `input_type=PCAP` 时使用。 |
| `pcap_repeat` / `pcap_rate` | 是否循环回放，以及回放倍率。 |
| `echo_option` | 回波模式：`0` 双回波合并（默认），`1` 仅第一回波，`2` 仅第二回波，`3` 双回波输出。 |

### 回波选择

通过 `lidar_sdk_config_t.echo_option` 选择点云回波输出模式：

```c
config.echo_option = 0;  /* 0=合并，1=第一回波，2=第二回波，3=双回波 */
```

- `0`：双回波参与处理，合并后输出一个回波；
- `1`：只处理并输出第一回波；
- `2`：只处理并输出第二回波；
- `3`：双回波参与处理并同时输出，回调帧的 `is_dual_return` 为非零。

默认值为 `0`。非法值（大于 `3`）会使 `lidar_driver_init()` 返回 `LIDAR_SDK_INVALID_ARGUMENT`。该配置由 SDK API 控制，不读取标定 JSON 中历史遗留的 `crosstalk_filter.echo_option` 字段。

标定文件路径示例。设置后不依赖默认的 `share/lidar_sdk/config/` 目录：

```c
snprintf(config.calibration_path,
         sizeof(config.calibration_path),
         "/opt/lidar_application/config/P60-Correction.json");
```

广播在线示例：

```c
config.model = LIDAR_SDK_MODEL_V192;
config.msop_port = 8899;
config.difop_port = 0;
strncpy(config.host_address, "0.0.0.0", sizeof(config.host_address) - 1U);
strncpy(config.group_address, "0.0.0.0", sizeof(config.group_address) - 1U);
```

单播在线示例（雷达端必须已配置向本机接收 IP、端口发送数据）：

```c
config.model = LIDAR_SDK_MODEL_P60_IMU;
config.msop_port = 8899;
config.difop_port = 0;
strncpy(config.host_address, "192.168.1.7", sizeof(config.host_address) - 1U);
```

PCAP 回放示例：

```c
config.model = LIDAR_SDK_MODEL_P60_IMU;
config.input_type = LIDAR_SDK_INPUT_PCAP;
config.msop_port = 8899;
config.pcap_repeat = 0;
config.pcap_rate = 1.0F;
strncpy(config.pcap_path, "/data/p60.pcap", sizeof(config.pcap_path) - 1U);
```

## 点云回调

SDK 在回调期间提供 `lidar_sdk_frame_t` 及其点数组。帧和点数组内存由 SDK 管理，仅在该次回调执行期间有效；如业务需要异步处理，请在回调中复制所需数据到业务自己的队列。

标准点结构 `lidar_sdk_point_t` 的坐标单位为米，`intensity` 为反射率/强度值，`ring` 为线束号，`time` 为该点相对当前帧首点的秒级偏移。`frame.timestamp` 是帧首点绝对时间（设备未提供有效时间戳时由主机时钟填充）。

## IMU 回调（P60_IMU）

P60_IMU 型号内嵌 IMU 传感器，SDK 会在每个数据包中解析 IMU 数据并通过独立回调输出。使用 `lidar_driver_set_imu_callback()` 注册 IMU 回调，回调在 SDK 内部线程中触发：

```c
static void on_imu(const lidar_sdk_imu_t* imu, void* user_data) {
  (void)user_data;
  printf("IMU ts=%.6f accel=(%.3f, %.3f, %.3f) gyro=(%.3f, %.3f, %.3f) temp=%.1f\n",
         imu->timestamp,
         imu->accel_x, imu->accel_y, imu->accel_z,
         imu->gyro_x, imu->gyro_y, imu->gyro_z,
         imu->temperature);
}

config.model = LIDAR_SDK_MODEL_P60_IMU;
lidar_driver_set_imu_callback(driver, on_imu, NULL);
```

`lidar_sdk_imu_t` 包含时间戳（`timestamp`）、三轴加速度（`accel_x/y/z`）、三轴角速度（`gyro_x/y/z`）和温度（`temperature`）。IMU 回调与点云回调相互独立，P60_IMU 同时输出点云和 IMU 数据；对 V192/P60 型号注册 IMU 回调不会报错，但不会触发。

## 线程模型

SDK 内部负责收包、解码、按帧处理、滤波和回调派发。应用程序不需要包含或配置内部队列、模板点云类型和工作线程。点云回调运行在 SDK 的派发线程中，应快速返回；耗时业务逻辑请投递到应用自己的线程。

调用顺序为：`create → set callbacks → init → start → stop → destroy`。`start`/`stop` 可用于控制本地接收和处理生命周期；它们不会向雷达下发启停出点命令。

## 支持范围

- 在线 UDP 输入与 PCAP 输入
- P60、P60_IMU、V192 解码和配套标定文件
- 内置软件滤波库
- Linux x86-64 与 ARM64 预编译算法库

雷达端启动/停止出点等控制协议将在雷达协议确定后作为独立异步控制接口加入；当前 `lidar_driver_start` / `lidar_driver_stop` 仅控制本地 SDK 的接收和处理生命周期。

## LiDAR-IMU 坐标系与外参

对于带 IMU 的产品（如 P60_IMU），统一采用以下定义：

- LiDAR 坐标系：`lidar`
- IMU 坐标系：`imu_link`
- 两个坐标系均为右手坐标系。
- IMU 原点在 LiDAR 坐标系中的位置：`[0.0000, 0.0183, -0.0345] m`。
- 旋转四元数：`[0.0, 0.0, 0.0, 1.0]`，顺序为 `x, y, z, w`。

外参变换约定为：

```text
p_lidar = R_lidar_imu · p_imu + t_lidar_imu
```

上述参数为统一默认外参。SDK 核心库仅负责传感器数据接收、解码和回调，不对外发布 ROS 消息或坐标系变换。

