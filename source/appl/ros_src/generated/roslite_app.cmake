# [note] Auto-generated file
# [note] 2019-03-28T05:27:45Z
# [note] based on source/appl/ros_src/map/roslite_map_two_listenres.map

#/****************************************************************************
 #[ roslite_app.cmake ] - cmake rules for roslite application
#****************************************************************************/


aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/nodes/talker talker_SOURCE)
add_library(
        ros_node-1-talker OBJECT
        ${talker_SOURCE}
)
target_compile_definitions(
        ros_node-1-talker
        PUBLIC ROSLITE_TARGET_CLUSTER_ID=1
        PUBLIC talker_MAIN=1
)

add_library(
        ros_node-1 STATIC
        $<TARGET_OBJECTS:ros_node-1-talker>
)

target_compile_definitions(
        ros_node-1
        PUBLIC ROSLITE_TARGET_CLUSTER_ID=1
)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/nodes/listener listener_SOURCE)
add_library(
        ros_node-2-listener OBJECT
        ${listener_SOURCE}
)
target_compile_definitions(
        ros_node-2-listener
        PUBLIC ROSLITE_TARGET_CLUSTER_ID=2
        PUBLIC listener_MAIN=1
)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/nodes/listener2 listener2_SOURCE)
add_library(
        ros_node-2-listener2 OBJECT
        ${listener2_SOURCE}
)
target_compile_definitions(
        ros_node-2-listener2
        PUBLIC ROSLITE_TARGET_CLUSTER_ID=2
        PUBLIC listener2_MAIN=1
)

add_library(
        ros_node-2 STATIC
        $<TARGET_OBJECTS:ros_node-2-listener>
        $<TARGET_OBJECTS:ros_node-2-listener2>
)

target_compile_definitions(
        ros_node-2
        PUBLIC ROSLITE_TARGET_CLUSTER_ID=2
)
