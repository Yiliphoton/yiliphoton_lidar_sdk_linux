cmake_minimum_required(VERSION 3.5)
project(ylp_driver_demos)

message("-- Building demos")
include_directories(${DRIVER_INCLUDE_DIRS})

set(YLP_CONFIG_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../src/ylp_driver/config)
set(YLP_CONFIG_FILES
    ${YLP_CONFIG_DIR}/P60-Correction.json
    ${YLP_CONFIG_DIR}/V192-Correction.json
)

add_executable(demo_online demo_online.cpp)
target_link_libraries(demo_online ${EXTERNAL_LIBS})

add_executable(demo_online_multi_lidars demo_online_multi_lidars.cpp)
target_link_libraries(demo_online_multi_lidars ${EXTERNAL_LIBS})

if(NOT DISABLE_PCAP_PARSE)
  add_executable(demo_pcap demo_pcap.cpp)
  target_link_libraries(demo_pcap ${EXTERNAL_LIBS})
endif()

# Copy calibration files next to each demo binary after build
foreach(_target demo_online demo_online_multi_lidars)
  add_custom_command(TARGET ${_target} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${YLP_CONFIG_FILES} $<TARGET_FILE_DIR:${_target}>
    COMMENT "Copying calibration files for ${_target}"
  )
endforeach()

if(NOT DISABLE_PCAP_PARSE)
  add_custom_command(TARGET demo_pcap POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${YLP_CONFIG_FILES} $<TARGET_FILE_DIR:demo_pcap>
    COMMENT "Copying calibration files for demo_pcap"
  )
endif()

# Install rules: binaries + calibration files go to the same directory
install(TARGETS demo_online demo_online_multi_lidars RUNTIME DESTINATION bin)
if(NOT DISABLE_PCAP_PARSE)
  install(TARGETS demo_pcap RUNTIME DESTINATION bin)
endif()
install(FILES ${YLP_CONFIG_FILES} DESTINATION bin)
