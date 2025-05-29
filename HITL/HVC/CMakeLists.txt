cmake_minimum_required(VERSION 3.24)

set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)

list(APPEND CMAKE_MODULE_PATH "{{sr:cmake_path}}")
message("Build CMAKE_MODULE_PATH: " ${CMAKE_MODULE_PATH})
include("cmake/gcc-arm-none-eabi.cmake")
message("Build CMAKE_MODULE_PATH: " ${CMAKE_MODULE_PATH})

project(HVC_HITL)
enable_language(C CXX ASM)
message("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_STANDARD 23)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)
set(CMAKE_COLOR_DIAGNOSTICS ON)

set(cpu_PARAMS
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
)

set(PROCESSOR F413)
set(PROCESSOR_FAM F4XX)

set(include_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc
)

set(include_c_DIRS)
set(include_cxx_DIRS)
set(include_asm_DIRS)

set(symbols_SYMB
    STM32F413xx
    ${PROCESSOR}
    HVC_HITL
)

add_library(freertos_config INTERFACE)
target_include_directories(freertos_config SYSTEM
    INTERFACE
    ${CMAKE_SOURCE_DIR}/Inc
    ${CMAKE_SOURCE_DIR}/../stm32f413-drivers/CMR/include
)
target_compile_definitions(freertos_config INTERFACE
    configASSERT
    projCOVERAGE_TEST=0
)

set(FREERTOS_PORT "GCC_ARM_CM4F" CACHE STRING "" FORCE)

set(symbols_c_SYMB
    configASSERT
)
set(symbols_cxx_SYMB)
set(symbols_asm_SYMB)

set(link_DIRS)
set(link_LIBS
    stm-drivers
)

set(linker_script_SRC "${CMAKE_CURRENT_SOURCE_DIR}/STM32F413xG_FLASH.ld")

set(compiler_OPTS
    -mfp16-format=ieee
    --specs=nano.specs
)

set(linker_OPTS)

include("cmake/vscode_generated.cmake")

link_directories(${CMAKE_PROJECT_NAME} ${link_DIRS})

add_executable(${CMAKE_PROJECT_NAME}
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/can.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/gpio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/i2c.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/pwm.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/syscall.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/sysmem.c
    # add any other source files here from Src/
)

target_sources(${CMAKE_PROJECT_NAME} PUBLIC)_