# LiDAR SDK 源码包快速开始

本文档面向只拿到 `lidar_sdk-<版本>-source.tar.gz` 的应用开发者。请在目标 Linux 设备上从源码构建 SDK；不要从其他架构的二进制包复制 `.so` 文件。

## 1. 解压与构建

Ubuntu/Debian 环境先安装依赖：

```bash
sudo apt update
sudo apt install -y build-essential cmake libeigen3-dev libpcap-dev
```

解压并安装 SDK：

```bash
tar -xzf lidar_sdk-<版本>-source.tar.gz
cd lidar_sdk-<版本>-source

cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
cmake --install build --prefix /opt/lidar_sdk
```

原生 x86-64 构建自动使用 `lib/x86-64/liblidar_algo.so`；原生 ARM64 构建自动使用 `lib/arm64/liblidar_algo.so`。安装完成后需要保留：

```text
/opt/lidar_sdk/lib/liblidar_sdk.so
/opt/lidar_sdk/lib/liblidar_algo.so
```

## 2. 运行在线 demo

`demo/demo.c` 是最小在线收包示例。默认配置为：

```text
型号：P60_IMU
MSOP 端口：8899
本机绑定地址：0.0.0.0（接收广播）
```

安装完成后直接运行：

```bash
/opt/lidar_sdk/bin/demo
```

收到点云后会持续打印帧序号、点数和时间戳；按 Enter 停止。

使用 V192 时，将 `demo/demo.c` 中的型号改为：

```c
config.model = LIDAR_SDK_MODEL_V192;
```

单播时，雷达端必须已经配置为向本机接收网卡 IP 和端口发送数据；将本机绑定 IP 改为该网卡 IP，例如：

```c
snprintf(config.host_address,
         sizeof(config.host_address),
         "192.168.1.7");
```

修改 demo 后重新执行 `cmake --build build -j` 和 `cmake --install build --prefix /opt/lidar_sdk`。

## 3. 验证客户工程链接

`examples/consumer_cmake/` 是独立客户工程最小示例，用于验证公开头文件、`find_package` 和动态库加载：

```bash
cmake -S examples/consumer_cmake -B consumer-build \
  -DCMAKE_PREFIX_PATH=/opt/lidar_sdk
cmake --build consumer-build -j

export LD_LIBRARY_PATH=/opt/lidar_sdk/lib:${LD_LIBRARY_PATH}
./consumer-build/lidar_sdk_consumer
```

该示例只验证 SDK 能被客户工程编译、链接和加载；实际收点云请按下一节注册回调并启动驱动。

## 4. 应用调用顺序

应用只包含公开头文件：

```c
#include <lidar_sdk/lidar_sdk.h>
```

标准调用顺序：

```text
lidar_sdk_config_init
lidar_driver_create
lidar_driver_set_frame_callback
lidar_driver_set_error_callback
lidar_driver_init
lidar_driver_start
lidar_driver_stop
lidar_driver_destroy
```

完整 C 调用示例见 [README.md](README.md) 的“应用集成”。点云通过 `lidar_sdk_frame_t` 回调输出；回调运行在 SDK 线程中，应快速返回，将耗时业务投递到应用自己的线程。

## 5. 常用配置

```c
lidar_sdk_config_t config;
lidar_sdk_config_init(&config);

config.model = LIDAR_SDK_MODEL_P60_IMU;
config.msop_port = 8899;
config.difop_port = 0;
config.echo_option = 0;  /* 0=合并，1=第一回波，2=第二回波，3=双回波 */
```

| 场景 | 需要设置的字段 |
| --- | --- |
| 广播在线 | `host_address = "0.0.0.0"`、`group_address = "0.0.0.0"`。 |
| 单播在线 | `host_address` 为本机接收网卡 IP；雷达端必须已向该 IP 与 `msop_port` 发送。 |
| V192 | `model = LIDAR_SDK_MODEL_V192`。 |
| PCAP 回放 | `input_type = LIDAR_SDK_INPUT_PCAP`，并设置 `pcap_path` 绝对路径。 |
| 自定义标定文件 | 设置 `calibration_path` 为 P60 或 V192 JSON 的绝对路径。 |
| 距离过滤 | 默认范围 `0.0~300.0m`，一般无需设置；自定义时 `max_distance_m` 必须大于 `min_distance_m`。 |
| 回波选择 | 设置 `echo_option`：`0` 双回波合并（默认），`1` 第一回波，`2` 第二回波，`3` 双回波输出。 |

`echo_option` 由 SDK API 配置，适用于 P60、P60_IMU 和 V192；修改后重新初始化驱动生效。`0` 和 `3` 会处理两个回波，`1` 和 `2` 只处理对应的单回波。详细行为见 [README.md](README.md) 的“回波选择”章节。

例如，标定 JSON 放在应用自己的配置目录时：

```c
snprintf(config.calibration_path,
         sizeof(config.calibration_path),
         "/opt/lidar_application/config/P60-Correction.json");
```

未设置 `calibration_path` 时，SDK 使用安装目录中的默认 JSON。

## 6. 常见问题

| 现象 | 检查方法 |
| --- | --- |
| `undefined reference to lidar_driver_*` | 确认最终可执行程序通过 `target_link_libraries(... lidar_sdk::lidar_sdk)` 链接 SDK，而不只是包含头文件。 |
| 启动时找不到 `liblidar_algo.so` | 确认它与 `liblidar_sdk.so` 位于同一库目录，或设置 `LD_LIBRARY_PATH=/opt/lidar_sdk/lib`。 |
| demo 没有点云输出 | 确认型号、`msop_port`、本机 IP、雷达发送目标和防火墙配置；广播使用 `0.0.0.0`。 |
| 标定文件打不开 | 设置 `calibration_path` 为存在的绝对路径，或保留 `/opt/lidar_sdk/share/lidar_sdk/config/` 默认文件。 |

更多 API 字段、PCAP 示例、IMU 回调和线程说明见 [README.md](README.md)。
