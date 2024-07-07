cmake_minimum_required(VERSION 3.15)

project(HAL)
message(STATUS "HAL")
#Not recomendded to use GLOB but this allows us to flexibly change HAL source
file(GLOB SOURCES CONFIGURE_DEPENDS ${PROCESSOR_FAM}/Src/*.c)
add_library(HAL)
target_sources(HAL PUBLIC ${SOURCES})
message("Processor Family: " ${PROCESSOR_FAM})
target_include_directories(HAL PUBLIC ${PROCESSOR_FAM}/Inc)



