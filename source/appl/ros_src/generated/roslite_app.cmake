# [note] Auto-generated file
# [note] 2019-03-28T05:27:45Z
# [note] based on source/appl/ros_src/map/roslite_map_two_listenres.map

#/****************************************************************************
 #[ roslite_app.cmake ] - cmake rules for roslite application
#****************************************************************************/

foreach(cluster_id 1 2)
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