# [note] Auto-generated file
# [note] 2019-04-03T05:14:19Z
# [note] based on source/appl/ros_src/map/roslite_map_two_listenres.map

foreach(cluster_id 0 1 2)
    add_library(
            roslite-${cluster_id} STATIC
            config/posix/src/roslite.cpp
            src/generated/init.cpp
            src/debug.cpp
            src/info.cpp
            src/init.cpp
            src/node_handle.cpp
            src/publisher.cpp
            src/rate.cpp
            src/subscriber.cpp
            src/thread.cpp
            src/time.cpp
            src/XmlRpcValue.cpp
            src/XmlRpcUtil.cpp
    )
    target_compile_definitions(
            roslite-${cluster_id}
            PUBLIC ROSLITE_TARGET_CLUSTER_ID=${cluster_id}
    )
    if(${cluster_id} EQUAL 0)
        find_package(xmlrpcpp REQUIRED)
        target_include_directories(
                roslite-${cluster_id}
                PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/..
                PRIVATE ${ros_INCLUDE_DIRS}
                PRIVATE ${xmlrpcpp_INCLUDE_DIRS}
        )
    else()
        target_include_directories(
                roslite-${cluster_id}
                PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
        )
    endif()
endforeach()