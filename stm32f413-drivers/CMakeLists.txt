cmake_minimum_required(VERSION 3.24)
project(stm-drivers)
message(STATUS "stm-drivers")
add_compile_definitions(${PROCESSOR})
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/DIM-ESE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/HAL)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/CMR)
add_library(stm-drivers STATIC)

target_link_libraries(CMR PUBLIC DIM-ESE freertos_kernel HAL)
target_link_libraries(stm-drivers
    PUBLIC
    CMR
)
