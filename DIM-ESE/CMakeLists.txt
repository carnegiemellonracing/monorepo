cmake_minimum_required(VERSION 3.15)

project(DIM-ESE)
message(STATUS "DIM-ESE")

add_library(DIM-ESE INTERFACE)
# There seems to be a way to keep the DIM-ESE/ on includes (install??)
# TODO properly structure DIM-ESE
target_include_directories(DIM-ESE INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})


