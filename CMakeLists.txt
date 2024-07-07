cmake_minimum_required(VERSION 3.15)
project(stm-drivers)
message(STATUS "stm-drivers")
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/DIM-ESE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/HAL)
add_library(stm-drivers INTERFACE)

target_link_libraries(stm-drivers
    INTERFACE
    DIM-ESE
    freertos_kernel
    HAL
)
