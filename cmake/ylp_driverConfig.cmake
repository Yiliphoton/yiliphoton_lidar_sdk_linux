cmake_minimum_required(VERSION 3.5)
include(CMakeFindDependencyMacro)

find_dependency(Eigen3 REQUIRED)
find_dependency(PkgConfig)
if(PkgConfig_FOUND)
  pkg_check_modules(PCAP libpcap)
endif()
if(NOT PCAP_FOUND)
  find_library(PCAP_LIBRARIES pcap)
endif()

# Reconstruct the prebuilt software-filter library relative to this file's install location.
# Layout after install:
#   <prefix>/lib/cmake/ylp_driver/ylp_driverConfig.cmake  ← this file
#   <prefix>/lib/libaglidar_ag1.so
if(NOT TARGET ylp_driver::aglidar_ag1)
  get_filename_component(_ylp_prefix "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)
  set(_ylp_ag1 "${_ylp_prefix}/lib/libaglidar_ag1.so")
  if(EXISTS "${_ylp_ag1}")
    add_library(ylp_driver::aglidar_ag1 SHARED IMPORTED)
    set_target_properties(ylp_driver::aglidar_ag1 PROPERTIES
      IMPORTED_LOCATION "${_ylp_ag1}")
  endif()
endif()

include("${CMAKE_CURRENT_LIST_DIR}/ylp_driverTargets.cmake")
