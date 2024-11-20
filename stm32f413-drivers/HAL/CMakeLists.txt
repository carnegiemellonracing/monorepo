cmake_minimum_required(VERSION 3.24)

project(HAL)

# Not recomended to use GLOB but this allows us to flexibly change HAL source
add_library(HAL)
file(GLOB SOURCES CONFIGURE_DEPENDS ${PROCESSOR_FAM}/Src/*.c)
file(GLOB SOURCES_CMSIS CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR}/stm32f413-drivers/CMSIS/*.c)

target_sources(HAL PUBLIC
${SOURCES}
${SOURCES_CMSIS}
${CMAKE_SOURCE_DIR}/stm32f413-drivers/CMSIS/system_stm32f4xx.c
)
message("Processor Family: " ${PROCESSOR_FAM})

target_include_directories(HAL PUBLIC
    ${CMAKE_SOURCE_DIR}/Inc #stm32f4xx_hal_conf.h
    ${PROCESSOR_FAM}/Inc
    ${PROCESSOR_FAM}/Inc/Legacy
    ${CMAKE_SOURCE_DIR}/stm32f413-drivers/CMSIS
    ${CMAKE_SOURCE_DIR}/stm32f413-drivers/CMSIS/Include
    ${CMAKE_SOURCE_DIR}/stm32f413-drivers/CMSIS/Device/${PROCESSOR})
