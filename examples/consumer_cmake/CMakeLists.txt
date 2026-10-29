cmake_minimum_required(VERSION 3.16)
project(lidar_sdk_consumer LANGUAGES C)

find_package(lidar_sdk CONFIG REQUIRED)

add_executable(lidar_sdk_consumer main.c)
target_link_libraries(lidar_sdk_consumer PRIVATE lidar_sdk::lidar_sdk)
