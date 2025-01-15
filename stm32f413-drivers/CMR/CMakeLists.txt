cmake_minimum_required(VERSION 3.24)

project(CMR)

add_library(CMR)
target_include_directories(CMR PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_sources(CMR PUBLIC
${CMAKE_CURRENT_SOURCE_DIR}/src/adc.c
${CMAKE_CURRENT_SOURCE_DIR}/src/can.c
${CMAKE_CURRENT_SOURCE_DIR}/src/fdcan.c
${CMAKE_CURRENT_SOURCE_DIR}/src/config_screen_helper.c
${CMAKE_CURRENT_SOURCE_DIR}/src/config.c
${CMAKE_CURRENT_SOURCE_DIR}/src/dma.c
${CMAKE_CURRENT_SOURCE_DIR}/src/f413.c
${CMAKE_CURRENT_SOURCE_DIR}/src/h725.c
${CMAKE_CURRENT_SOURCE_DIR}/src/fir_filter.c
${CMAKE_CURRENT_SOURCE_DIR}/src/gpio.c
${CMAKE_CURRENT_SOURCE_DIR}/src/i2c.c
${CMAKE_CURRENT_SOURCE_DIR}/src/interrupts.c
${CMAKE_CURRENT_SOURCE_DIR}/src/panic.c
${CMAKE_CURRENT_SOURCE_DIR}/src/pwm.c
${CMAKE_CURRENT_SOURCE_DIR}/src/qspi.c
${CMAKE_CURRENT_SOURCE_DIR}/src/rcc.c
${CMAKE_CURRENT_SOURCE_DIR}/src/sensors.c
${CMAKE_CURRENT_SOURCE_DIR}/src/spi.c
${CMAKE_CURRENT_SOURCE_DIR}/src/tasks.c
${CMAKE_CURRENT_SOURCE_DIR}/src/uart.c
${CMAKE_CURRENT_SOURCE_DIR}/src/watchdog.c
${CMAKE_CURRENT_SOURCE_DIR}/src/openblt.c
)

