cmake_minimum_required(VERSION 3.16)
set(LIDAR_SDK_VERSION "1.1.1" CACHE STRING "LiDAR SDK release version")
project(lidar_sdk VERSION ${LIDAR_SDK_VERSION} LANGUAGES C CXX)

include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(DISABLE_PCAP_PARSE "Disable PCAP file parsing" OFF)
option(ENABLE_TRANSFORM "Enable point transform" OFF)
option(ENABLE_DOUBLE_RCVBUF "Double UDP receive buffer size" ON)
option(ENABLE_THREAD_PRIORITY "Raise receive and processing thread priority" OFF)
option(ENABLE_CRC32_CHECK "Enable CRC32 checking on MSOP packets" OFF)
option(ENABLE_POINT_EXTEND_FIELD "Enable azimuth/elevation fields internally" OFF)
option(BUILD_DEMO "Build the public API demo" ON)
option(BUILD_PCAP_TEST "Build the PCAP smoke test" OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wno-unused-parameter)
endif()

set(SDK_INTERNAL_INCLUDE_DIR ${CMAKE_CURRENT_LIST_DIR}/src)

find_package(Eigen3 REQUIRED)

if(NOT DISABLE_PCAP_PARSE)
  find_library(PCAP_LIBRARY pcap REQUIRED)
endif()

string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" LIDAR_SDK_TARGET_PROCESSOR)
if(LIDAR_SDK_TARGET_PROCESSOR MATCHES "^(x86_64|amd64)$")
  set(LIB_ARCH_DIR x86-64)
  set(LIDAR_SDK_PACKAGE_ARCH x86_64)
elseif(LIDAR_SDK_TARGET_PROCESSOR MATCHES "^(aarch64|arm64)$")
  set(LIB_ARCH_DIR arm64)
  set(LIDAR_SDK_PACKAGE_ARCH aarch64)
else()
  message(FATAL_ERROR
    "Unsupported target processor: ${CMAKE_SYSTEM_PROCESSOR}. "
    "Supported processors are x86_64/amd64 and aarch64/arm64.")
endif()

find_library(LIDAR_ALGO_LIB lidar_algo PATHS ${CMAKE_CURRENT_LIST_DIR}/lib/${LIB_ARCH_DIR} NO_DEFAULT_PATH REQUIRED)
add_library(lidar_algo SHARED IMPORTED GLOBAL)
set_target_properties(lidar_algo PROPERTIES IMPORTED_LOCATION "${LIDAR_ALGO_LIB}")

set(SDK_PRIVATE_DEFINITIONS)
if(DISABLE_PCAP_PARSE)
  list(APPEND SDK_PRIVATE_DEFINITIONS DISABLE_PCAP_PARSE)
endif()
if(ENABLE_TRANSFORM)
  list(APPEND SDK_PRIVATE_DEFINITIONS ENABLE_TRANSFORM)
endif()
if(ENABLE_DOUBLE_RCVBUF)
  list(APPEND SDK_PRIVATE_DEFINITIONS ENABLE_DOUBLE_RCVBUF)
endif()
if(ENABLE_THREAD_PRIORITY)
  list(APPEND SDK_PRIVATE_DEFINITIONS ENABLE_THREAD_PRIORITY HIGH_THREAD_PRIORITY=11)
endif()
if(ENABLE_CRC32_CHECK)
  list(APPEND SDK_PRIVATE_DEFINITIONS ENABLE_CRC32_CHECK)
endif()
if(ENABLE_POINT_EXTEND_FIELD)
  list(APPEND SDK_PRIVATE_DEFINITIONS ENABLE_POINT_EXTEND_FIELD)
endif()

set(LIDAR_SDK_GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/version.hpp.in"
  "${LIDAR_SDK_GENERATED_DIR}/core/common/version.hpp"
  @ONLY)

add_library(lidar_sdk SHARED
  src/api/lidar_sdk.cpp
  src/core/driver/decoder/reflectivity_compensation.cpp
)
add_library(lidar_sdk::lidar_sdk ALIAS lidar_sdk)

target_compile_definitions(lidar_sdk PRIVATE
  LIDAR_SDK_BUILDING_LIBRARY
  ${SDK_PRIVATE_DEFINITIONS}
)
target_include_directories(lidar_sdk
  PRIVATE ${LIDAR_SDK_GENERATED_DIR} ${SDK_INTERNAL_INCLUDE_DIR}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(lidar_sdk PRIVATE
  Eigen3::Eigen
  ${CMAKE_DL_LIBS}
  pthread
  lidar_algo
)
if(NOT DISABLE_PCAP_PARSE)
  target_link_libraries(lidar_sdk PRIVATE ${PCAP_LIBRARY})
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(lidar_sdk PRIVATE -fvisibility=hidden)
endif()
set_target_properties(lidar_sdk PROPERTIES
  EXPORT_NAME lidar_sdk
  VERSION ${PROJECT_VERSION}
  SOVERSION 1
  BUILD_RPATH "$ORIGIN"
  INSTALL_RPATH "$ORIGIN"
)

set(LIDAR_SDK_BUILD_INFO "${CMAKE_CURRENT_BINARY_DIR}/BUILD-INFO.txt")
file(WRITE "${LIDAR_SDK_BUILD_INFO}"
  "LiDAR SDK Build Information\n"
  "Version: ${PROJECT_VERSION}\n"
  "Target system: ${CMAKE_SYSTEM_NAME}\n"
  "Target processor: ${CMAKE_SYSTEM_PROCESSOR}\n"
  "Package architecture: ${LIDAR_SDK_PACKAGE_ARCH}\n"
  "C compiler: ${CMAKE_C_COMPILER_ID} ${CMAKE_C_COMPILER_VERSION}\n"
  "C++ compiler: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}\n"
  "Build type: ${CMAKE_BUILD_TYPE}\n")

if(BUILD_PCAP_TEST)
  add_executable(lidar_sdk_pcap_test tests/pcap_smoke_test.c)
  target_link_libraries(lidar_sdk_pcap_test PRIVATE lidar_sdk)
  set_target_properties(lidar_sdk_pcap_test PROPERTIES
    LINKER_LANGUAGE CXX
    INSTALL_RPATH "$ORIGIN/../${CMAKE_INSTALL_LIBDIR}"
  )
endif()

if(BUILD_DEMO)
  add_executable(demo demo/demo.c)
  target_link_libraries(demo PRIVATE lidar_sdk)
  set_target_properties(demo PROPERTIES
    LINKER_LANGUAGE CXX
    INSTALL_RPATH "$ORIGIN/../${CMAKE_INSTALL_LIBDIR}"
  )
endif()

install(DIRECTORY include/lidar_sdk DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS lidar_sdk
  EXPORT lidar_sdkTargets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(FILES ${LIDAR_ALGO_LIB} DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(FILES
  resources/calibration/P60-Correction.json
  resources/calibration/V192-Correction.json
  DESTINATION ${CMAKE_INSTALL_DATADIR}/lidar_sdk/config)
install(EXPORT lidar_sdkTargets
  FILE lidar_sdkTargets.cmake
  NAMESPACE lidar_sdk::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/lidar_sdk)
install(FILES cmake/lidar_sdkConfig.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/lidar_sdk)
install(FILES README.md README_EN.md ${LIDAR_SDK_BUILD_INFO}
  DESTINATION ${CMAKE_INSTALL_DOCDIR})
install(FILES README.md README_EN.md
  DESTINATION .)
install(FILES QUICKSTART_BINARY.md
  DESTINATION .
  RENAME QUICKSTART.md)
install(FILES QUICKSTART_BINARY_EN.md
  DESTINATION .
  RENAME QUICKSTART_EN.md)
install(FILES QUICKSTART_BINARY.md
  DESTINATION ${CMAKE_INSTALL_DOCDIR}
  RENAME QUICKSTART.md)
install(FILES QUICKSTART_BINARY_EN.md
  DESTINATION ${CMAKE_INSTALL_DOCDIR}
  RENAME QUICKSTART_EN.md)
install(DIRECTORY examples/consumer_cmake
  DESTINATION ${CMAKE_INSTALL_DATADIR}/lidar_sdk/examples
  FILES_MATCHING PATTERN "CMakeLists.txt" PATTERN "*.c" PATTERN "*.h" PATTERN "*.md")
if(BUILD_DEMO)
  install(TARGETS demo RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()

string(TOLOWER "${CMAKE_SYSTEM_NAME}" LIDAR_SDK_PACKAGE_SYSTEM)
set(CPACK_GENERATOR "TGZ")
set(CPACK_PACKAGE_NAME "lidar_sdk")
set(CPACK_PACKAGE_VENDOR "LiDAR SDK")
set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
set(CPACK_PACKAGE_FILE_NAME
  "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}-${LIDAR_SDK_PACKAGE_SYSTEM}-${LIDAR_SDK_PACKAGE_ARCH}")
set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY ON)
set(CPACK_PACKAGING_INSTALL_PREFIX "/")
set(CPACK_RESOURCE_FILE_README "${CMAKE_CURRENT_LIST_DIR}/README_EN.md")
include(CPack)
