cmake_minimum_required(VERSION 3.15)

project(DIM-ESE)
message(STATUS "DIM-ESE")

add_library(DIM-ESE INTERFACE)

target_include_directories(DIM-ESE INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(DIM-ESE INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/content)
target_include_directories(DIM-ESE INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/images)




