cmake_minimum_required(VERSION 2.6)

include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/../roslite/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../roslite/config/posix/include
)

include(${CMAKE_CURRENT_SOURCE_DIR}/generated/roslite_app.cmake)
