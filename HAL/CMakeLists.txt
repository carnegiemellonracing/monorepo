cmake_minimum_required(VERSION 3.15)

project(HAL)
message(STATUS "HAL")

# Not recomended to use GLOB but this allows us to flexibly change HAL source
add_library(HAL)
file(GLOB SOURCES CONFIGURE_DEPENDS ${PROCESSOR_FAM}/Src/*.c)
target_sources(HAL PUBLIC ${SOURCES})
message("Processor Family: " ${PROCESSOR_FAM})
target_sources(HAL PUBLIC
${CMAKE_SOURCE_DIR}/stm32f413-drivers/CMSIS/system_stm32f4xx.c
)
target_include_directories(HAL PUBLIC ${CMAKE_SOURCE_DIR}/Inc
    ${PROCESSOR_FAM}/Inc
    ${PROCESSOR_FAM}/Inc/Legacy
    ${CMAKE_SOURCE_DIR}/stm32f413-drivers/CMSIS
    ${CMAKE_SOURCE_DIR}/stm32f413-drivers/CMSIS/Include
    ${CMAKE_SOURCE_DIR}/stm32f413-drivers/CMSIS/Device/${PROCESSOR})
