# SDK 版本与发布说明

本文档面向 SDK 维护者，说明版本定义、一键打包、交付文件和常见清理操作。

## 1. 版本定义

SDK 的发布版本只在 `CMakeLists.txt` 顶部定义一次：

```cmake
project(lidar_sdk VERSION 1.1.0 LANGUAGES C CXX)
```

发布新版本时，将 `1.1.0` 修改为目标版本，例如：

```cmake
project(lidar_sdk VERSION 1.1.1 LANGUAGES C CXX)
```

重新执行一键打包脚本后，版本会自动同步到：

- CPack 交付压缩包名称。
- 源码加二进制交付压缩包名称。
- `liblidar_sdk.so.<完整版本号>` 真实库文件名称。

### ABI 主版本

`CMakeLists.txt` 中的以下配置控制共享库 ABI 主版本：

```cmake
SOVERSION 1
```

- 仅修复问题、增加兼容功能或保持现有公开 C API 二进制兼容时，保持 `SOVERSION 1`。
- 删除、修改已有公开结构体布局或函数签名等 ABI 不兼容改动时，升为 `SOVERSION 2`，同时应升级发布版本。

不要为了普通功能迭代单独修改 `SOVERSION`。

## 2. 一键打包

在目标 Linux 架构环境中执行：

```bash
cd /home/zyy/sdk_release/lidar_driver_sdk
./scripts/package.sh
```

脚本会依次执行：

1. 使用 `release` CMake preset 配置并构建当前 Linux 架构的 Release SDK。
2. 安装到临时目录，并校验主库架构、公开 API 符号、算法库依赖、运行时 `$ORIGIN`、标定 JSON、demo 和 CMake 包配置。
3. 使用独立客户示例通过 `find_package(lidar_sdk)` 编译、链接并加载 SDK。
4. 使用 CPack 生成当前架构的标准二进制 SDK 包。
5. 生成不包含预编译 SDK 的独立源码包，并生成 SHA-256 校验文件。

任一校验失败时，脚本会退出，不应交付当次生成的压缩包。

## 3. 发布输出

打包结果位于：

```text
out/dist/
```

以版本 `1.1.0`、x86-64 为例：

```text
lidar_sdk-1.1.0-linux-x86_64.tar.gz
lidar_sdk-1.1.0-source.tar.gz
lidar_sdk-1.1.0-SHA256SUMS.txt
```

### 标准二进制包

`lidar_sdk-<版本>-linux-<架构>.tar.gz` 是默认交付给客户的包，包含：

```text
include/lidar_sdk/lidar_sdk.h       # 公开 C API
lib/liblidar_sdk.so*                # SDK 动态库及版本链接
lib/liblidar_algo.so                # 算法动态库
share/lidar_sdk/config/             # P60、V192 标定 JSON
lib/cmake/lidar_sdk/                # find_package 配置
bin/demo                            # 示例程序
share/doc/lidar_sdk/                # 中英文 README、BUILD-INFO.txt
```

二进制包只能用于压缩包名称标注的架构和兼容运行环境。客户正常开发可直接使用该包，无需编译 SDK 源码。

### 独立源码包

`lidar_sdk-<版本>-source.tar.gz` 是所有平台的构建基线，内容为完整项目根目录：

```text
include/、src/、resources/、lib/、cmake/、demo/、examples/、scripts/
```

源码包同时包含 `lib/x86-64/` 与 `lib/arm64/` 的私有算法库。客户应在目标设备的源码根目录原生构建；CMake 会自动选择匹配的算法库。源码包不夹带任何架构的预编译 `liblidar_sdk.so`，避免 ARM64 客户误用 x86-64 二进制库。

默认向已验证平台提供对应架构的二进制包；特殊系统、未发布二进制架构或需要定制时，交付源码包。

## 4. 目录清理规则

```text
build/                 # 旧手工构建缓存，可删除
out/build/             # 一键打包构建缓存，可删除后自动重新生成
out/dist/              # 最终交付压缩包；按发布保留或手工清理旧版本
package/               # 旧手工安装目录，不参与当前流程，可删除
```

不要手工从 `build/` 或 `package/` 复制文件交付。始终运行 `./scripts/package.sh`，并从 `out/dist/` 取得发布包。

## 5. 架构与环境

- x86-64 与 ARM64 二进制包必须在对应架构的 Linux 环境中构建和验证。
- 未验证的架构只交付源码包，不声明提供预编译二进制 SDK。
- 不同架构生成不同文件名，例如 `linux-x86_64` 或 `linux-aarch64`。
- 目标设备的 glibc 和 `libstdc++` 版本必须与构建环境兼容；若环境较旧，应在兼容的较旧系统中构建。
- `liblidar_sdk.so` 与 `liblidar_algo.so` 必须位于同一动态库搜索路径；标定 JSON 可通过 `calibration_path` 配置到任意目录。

## 6. 发布前检查

每次发布前确认：

1. `CMakeLists.txt` 中的 `VERSION` 已更新。
2. `./scripts/package.sh` 执行成功。
3. `out/dist/` 中存在当前架构的二进制包、独立源码包和 SHA-256 校验文件。
4. 检查 `BUILD-INFO.txt`、压缩包名称和实际库架构一致。
5. 不交付名称与内容可能不一致的旧压缩包。
6. 在目标架构或等价环境中用 demo / 客户示例完成一次加载验证。
