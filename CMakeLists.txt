cmake_minimum_required(VERSION 2.6)

project(ros_lite)

add_definitions("-std=c++11")

include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/source/appl/roslite/config/posix/include
)

add_subdirectory(source/appl/roslite)
add_subdirectory(source/appl/ros_src)

foreach(cluster_id RANGE 1 2)
    add_executable(
            cluster-${cluster_id}
            ${CMAKE_CURRENT_SOURCE_DIR}/source/appl/ros_src/generated/init_threads.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/source/appl/roslite/src/generated/init.cpp
    )
    target_include_directories(
            cluster-${cluster_id}
            PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/source/appl/roslite/include
    )
    target_compile_definitions(
            cluster-${cluster_id}
            PUBLIC ROSLITE_TARGET_CLUSTER_ID=${cluster_id}
    )
    target_link_libraries(
            cluster-${cluster_id}
            ros_node-${cluster_id}
            roslite-${cluster_id}
            pthread
    )
endforeach(cluster_id)

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
        PRIVATE ${ros_INCLUDE_DIRS}
)
target_compile_definitions(
        ros_bridge
        PUBLIC ROSLITE_TARGET_CLUSTER_ID=0
)
target_link_libraries(
        ros_bridge
        roslite-0
        pthread
        ${ros_LIBRARIES}
)
