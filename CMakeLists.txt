cmake_minimum_required(VERSION 2.6)

project(ros_lite)

add_definitions("-std=c++11")

include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/source/appl/roslite/config/posix/include
)

add_subdirectory(source/appl/roslite)
add_subdirectory(source/appl/ros_src)

include(${CMAKE_CURRENT_SOURCE_DIR}/source/appl/ros_src/generated/roslite_app.cmake)

find_package(roscpp REQUIRED)
find_package(xmlrpcpp REQUIRED)
include(${CMAKE_CURRENT_SOURCE_DIR}/source/appl/ros_bridge/generated/ros_bridge_generated.cmake)

add_executable(
        ros_bridge
        ${CMAKE_CURRENT_SOURCE_DIR}/source/appl/ros_bridge/ros_bridge.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/source/appl/ros_bridge/generated/ros_bridge_generated.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/source/appl/ros_src/generated/init_threads.cpp
)
target_include_directories(
        ros_bridge
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/source/appl
        PRIVATE ${ros_bridge_INCLUDE_DIRS}
        PRIVATE ${xmlrpcpp_INCLUDE_DIRS}
)
target_compile_definitions(
        ros_bridge
        PUBLIC ROSLITE_TARGET_CLUSTER_ID=0
)
target_link_libraries(
        ros_bridge
        roslite-0
        pthread
        ${ros_bridge_LIBRARIES}
        ${roscpp_LIBRARIES}
        ${xmlrpcpp_LIBRARIES}
)

add_executable(
        ros_param_bridge
        ${CMAKE_CURRENT_SOURCE_DIR}/source/appl/ros_param_bridge/ros_param_bridge.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/source/appl/ros_src/generated/init_threads.cpp
)
target_include_directories(
        ros_param_bridge
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/source/appl
        PRIVATE ${ros_bridge_INCLUDE_DIRS}
        PRIVATE ${xmlrpcpp_INCLUDE_DIRS}
)
target_compile_definitions(
        ros_param_bridge
        PUBLIC ROSLITE_TARGET_CLUSTER_ID=0
)
target_link_libraries(
        ros_param_bridge
        roslite-0
        pthread
        ${ros_bridge_LIBRARIES}
        ${roscpp_LIBRARIES}
        ${xmlrpcpp_LIBRARIES}
)
