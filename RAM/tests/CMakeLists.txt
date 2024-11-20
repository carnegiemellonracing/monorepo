cmake_minimum_required(VERSION 3.21)
project(MyUnitTests)

include(CTest)
enable_testing()

add_subdirectory(../stm32f413-drivers/Unity unity)
add_subdirectory(../cn-cbor cn-cbor)

include_directories()

add_executable(MyUnitTests test1.c)

target_link_libraries(MyUnitTests unity cn-cbor)
add_test(tests1 MyUnitTests)

