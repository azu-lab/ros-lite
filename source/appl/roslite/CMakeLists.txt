cmake_minimum_required(VERSION 2.6)

include(${CMAKE_CURRENT_SOURCE_DIR}/../ros_bridge/generated/ros_bridge_generated.cmake)

include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/config/posix/include
)

include(${CMAKE_CURRENT_SOURCE_DIR}/generated/roslite.cmake)
