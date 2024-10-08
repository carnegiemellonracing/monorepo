cmake_minimum_required(VERSION 3.24)

project(CMR)

add_library(CMR)
target_include_directories(CMR PUBLIC include)
target_sources(CMR PUBLIC
${CMAKE_SOURCE_DIR}/stm32f413-drivers/CMR/src/adc.c
${CMAKE_SOURCE_DIR}/stm32f413-drivers/CMR/src/can.c
${CMAKE_SOURCE_DIR}/stm32f413-drivers/CMR/src/config_screen_helper.c
${CMAKE_SOURCE_DIR}/stm32f413-drivers/CMR/src/config.c
${CMAKE_SOURCE_DIR}/stm32f413-drivers/CMR/src/dma.c
${CMAKE_SOURCE_DIR}/stm32f413-drivers/CMR/src/dsp.c
${CMAKE_SOURCE_DIR}/stm32f413-drivers/CMR/src/f413.c
${CMAKE_SOURCE_DIR}/stm32f413-drivers/CMR/src/fir_filter.c
${CMAKE_SOURCE_DIR}/stm32f413-drivers/CMR/src/gpio.c
${CMAKE_SOURCE_DIR}/stm32f413-drivers/CMR/src/i2c.c
${CMAKE_SOURCE_DIR}/stm32f413-drivers/CMR/src/interrupts.c
${CMAKE_SOURCE_DIR}/stm32f413-drivers/CMR/src/l431.c
${CMAKE_SOURCE_DIR}/stm32f413-drivers/CMR/src/panic.c
${CMAKE_SOURCE_DIR}/stm32f413-drivers/CMR/src/pwm.c
${CMAKE_SOURCE_DIR}/stm32f413-drivers/CMR/src/qspi.c
${CMAKE_SOURCE_DIR}/stm32f413-drivers/CMR/src/rcc.c
${CMAKE_SOURCE_DIR}/stm32f413-drivers/CMR/src/spi.c
${CMAKE_SOURCE_DIR}/stm32f413-drivers/CMR/src/tasks.c
${CMAKE_SOURCE_DIR}/stm32f413-drivers/CMR/src/uart.c
${CMAKE_SOURCE_DIR}/stm32f413-drivers/CMR/src/watchdog.c
)

