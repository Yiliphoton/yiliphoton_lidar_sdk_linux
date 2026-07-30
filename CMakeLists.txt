cmake_minimum_required(VERSION 3.5)
project(ylp_driver VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE Release)
endif()
message("-- Build type: ${CMAKE_BUILD_TYPE}")

# ============================================================
#  Options
# ============================================================
option(DISABLE_PCAP_PARSE        "Disable PCAP file parse"             OFF)
option(ENABLE_TRANSFORM          "Enable point transform (Eigen)"       OFF)
option(ENABLE_MOTION_COMPENSATION "Enable IMU motion compensation"      OFF)
option(ENABLE_DOUBLE_RCVBUF      "Double UDP receive buffer size"       ON)
option(ENABLE_THREAD_PRIORITY    "Raise recv/process thread priority"   OFF)
option(ENABLE_PCL_POINTCLOUD     "Use PCL PointCloud type"              OFF)
option(ENABLE_CRC32_CHECK        "Enable CRC32 check on MSOP packets"   OFF)
option(ENABLE_POINT_EXTEND_FIELD "Enable azimuth/elevation in output"   OFF)
option(COMPILE_DEMOS             "Build demo programs"                  ON)
option(COMPILE_TOOLS             "Build tool programs"                  OFF)

# ============================================================
#  Platform
# ============================================================
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-unused-parameter")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
  set(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG} -g")
endif()

# ============================================================
#  Include paths
# ============================================================
set(DRIVER_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/src)
include_directories(${DRIVER_INCLUDE_DIRS})

# ============================================================
#  External dependencies
# ============================================================
list(APPEND EXTERNAL_LIBS pthread)

if(NOT DISABLE_PCAP_PARSE)
  list(APPEND EXTERNAL_LIBS pcap)
else()
  add_definitions(-DDISABLE_PCAP_PARSE)
endif()

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Software filter precompiled library
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|ARM64")
  set(YLP_LIB_ARCH_DIR arm64)
else()
  set(YLP_LIB_ARCH_DIR x86-64)
endif()
message("-- Target arch lib dir: ${YLP_LIB_ARCH_DIR}")

find_library(YLP_AG1_LIB aglidar_ag1 PATHS ${CMAKE_CURRENT_LIST_DIR}/lib/${YLP_LIB_ARCH_DIR})
if(YLP_AG1_LIB)
  add_library(_ylp_aglidar_ag1 SHARED IMPORTED GLOBAL)
  set_target_properties(_ylp_aglidar_ag1 PROPERTIES IMPORTED_LOCATION "${YLP_AG1_LIB}")
  list(APPEND EXTERNAL_LIBS ${YLP_AG1_LIB})
  message("-- Found software filter library: ${YLP_AG1_LIB}")
else()
  message(WARNING "Software filter library (libaglidar_ag1) not found")
endif()

# ============================================================
#  Feature flags
# ============================================================
if(ENABLE_TRANSFORM)
  add_definitions(-DENABLE_TRANSFORM)
endif()

if(ENABLE_MOTION_COMPENSATION)
  add_definitions(-DENABLE_MOTION_COMPENSATION)
endif()

if(ENABLE_DOUBLE_RCVBUF)
  add_definitions(-DENABLE_DOUBLE_RCVBUF)
endif()

if(ENABLE_THREAD_PRIORITY)
  add_definitions(-DENABLE_THREAD_PRIORITY)
  add_definitions(-DHIGH_THREAD_PRIORITY=11)
endif()

if(ENABLE_PCL_POINTCLOUD)
  add_definitions(-DENABLE_PCL_POINTCLOUD)
  find_package(PCL REQUIRED)
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})
  list(APPEND EXTERNAL_LIBS ${PCL_LIBRARIES})
endif()

if(ENABLE_CRC32_CHECK)
  add_definitions(-DENABLE_CRC32_CHECK)
endif()

if(ENABLE_POINT_EXTEND_FIELD)
  add_definitions(-DENABLE_POINT_EXTEND_FIELD)
endif()

# ============================================================
#  Core library (compiled .cpp files from the SDK)
# ============================================================
set(YLP_DRIVER_SOURCES
  src/ylp_driver/driver/decoder/reflectivity_compensation.cpp
)

add_library(ylp_driver_core STATIC ${YLP_DRIVER_SOURCES})
target_include_directories(ylp_driver_core PUBLIC
  $<BUILD_INTERFACE:${DRIVER_INCLUDE_DIRS}>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(ylp_driver_core PUBLIC
  pthread
  Eigen3::Eigen
  $<$<NOT:$<BOOL:${DISABLE_PCAP_PARSE}>>:pcap>
  $<BUILD_INTERFACE:$<$<BOOL:${YLP_AG1_LIB}>:_ylp_aglidar_ag1>>
  $<INSTALL_INTERFACE:$<$<BOOL:${YLP_AG1_LIB}>:ylp_driver::aglidar_ag1>>
)
add_library(ylp_driver::ylp_driver_core ALIAS ylp_driver_core)
list(APPEND EXTERNAL_LIBS ylp_driver_core)

# ============================================================
#  Sub-targets
# ============================================================
if(COMPILE_DEMOS)
  add_subdirectory(demo)
endif()

if(COMPILE_TOOLS)
  add_subdirectory(tool)
endif()

# ============================================================
#  Install rules — enables find_package(ylp_driver) for consumers
# ============================================================
include(GNUInstallDirs)

install(DIRECTORY src/ylp_driver DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
  PATTERN "config" EXCLUDE)

install(TARGETS ylp_driver_core
  EXPORT ylp_driverTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

if(YLP_AG1_LIB)
  install(FILES ${YLP_AG1_LIB} DESTINATION ${CMAKE_INSTALL_LIBDIR})
endif()

install(FILES
  src/ylp_driver/config/P60-Correction.json
  src/ylp_driver/config/V192-Correction.json
  DESTINATION ${CMAKE_INSTALL_DATADIR}/ylp_driver/config)

install(EXPORT ylp_driverTargets
  FILE ylp_driverTargets.cmake
  NAMESPACE ylp_driver::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ylp_driver)

install(FILES cmake/ylp_driverConfig.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ylp_driver)
