project(MyUnitTests)

cmake_minimum_required(VERSION 3.21)
message(STATUS "Configuring Tests")
add_subdirectory(${CMAKE_SOURCE_DIR}/stm32f413-drivers/Unity build)
include_directories(${CMAKE_SOURCE_DIR}/stm32f413-drivers/Unity/src)

add_executable(MyUnitTests test1.c)

target_link_libraries(MyUnitTests unity)
add_test(tests1 MyUnitTests)

