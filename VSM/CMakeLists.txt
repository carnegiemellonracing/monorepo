cmake_minimum_required(VERSION 3.24)

#
# User is free to modify the file as much as necessary
#
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)

# list(APPEND CMAKE_MODULE_PATH "C:/Users/user1/Downloads/CmakeTestProject/cmake")
list(APPEND CMAKE_MODULE_PATH "{{sr:cmake_path}}")
message("Build CMAKE_MODULE_PATH: " ${CMAKE_MODULE_PATH})

include("cmake/gcc-arm-none-eabi.cmake")

message("Build CMAKE_MODULE_PATH: " ${CMAKE_MODULE_PATH})


# Core project settings
project(VCU)
enable_language(C CXX ASM)

message("Build type: " ${CMAKE_BUILD_TYPE})


# Setup compiler settings
set(CMAKE_C_STANDARD 23)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

set(CMAKE_COLOR_DIAGNOSTICS ON)


# Generate git commit information
if(CMAKE_HOST_WIN32)
    add_custom_target(generate_gitcommit ALL
        COMMAND powershell -ExecutionPolicy Bypass
                -File ${CMAKE_CURRENT_SOURCE_DIR}/export_git.ps1
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        COMMENT "Generating gitcommit.h using PowerShell"
    )
elseif(CMAKE_HOST_UNIX)
    add_custom_target(generate_gitcommit ALL
        COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/export_git.sh
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        COMMENT "Generating gitcommit.h on Linux/Unix"
    )
endif()


# Core MCU flags
set(cpu_PARAMS
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
)


# Processor
set(sources_SRCS)

set(PROCESSOR F413)
set(PROCESSOR_FAM F4XX)


# Include directories
set(include_DIRS
    ${CMAKE_SOURCE_DIR}/Inc
    ${CMAKE_SOURCE_DIR}/optimizer
    ${CMAKE_SOURCE_DIR}/data
)

set(include_c_DIRS)
set(include_cxx_DIRS)
set(include_asm_DIRS)


# Symbols
set(symbols_SYMB
    STM32F413xx
    ${PROCESSOR}
)

set(symbols_c_SYMB
    configASSERT
)

set(symbols_cxx_SYMB)
set(symbols_asm_SYMB)


# FreeRTOS configuration
add_library(freertos_config INTERFACE)

target_include_directories(freertos_config SYSTEM
    INTERFACE
    ${CMAKE_SOURCE_DIR}/Inc
    ${CMAKE_SOURCE_DIR}/../stm32f413-drivers/CMR/include
)

target_compile_definitions(freertos_config
    INTERFACE
    configASSERT
    projCOVERAGE_TEST=0
)

set(FREERTOS_PORT "GCC_ARM_CM4F" CACHE STRING "" FORCE)


# Link directories and libraries
set(link_DIRS)

set(link_LIBS
    stm-drivers
)


# Linker script
set(linker_script_SRC)


# Compiler options
set(compiler_OPTS
    -mfp16-format=ieee
    --specs=nano.specs
)


# Linker options
set(linker_OPTS)


# Include generated CMake information
include("cmake/vscode_generated.cmake")


# Link directories setup
link_directories(${CMAKE_PROJECT_NAME} ${link_DIRS})


# Create executable
add_executable(${CMAKE_PROJECT_NAME}

    # =========================
    # VCU source files
    # =========================

    ${CMAKE_SOURCE_DIR}/Src/adc.c
    ${CMAKE_SOURCE_DIR}/Src/assi.c
    ${CMAKE_SOURCE_DIR}/Src/brakelight.c
    ${CMAKE_SOURCE_DIR}/Src/can.c
    ${CMAKE_SOURCE_DIR}/Src/cmr_error.c
    ${CMAKE_SOURCE_DIR}/Src/control_sensors.c
    ${CMAKE_SOURCE_DIR}/Src/controls_helper.c
    ${CMAKE_SOURCE_DIR}/Src/controls.c
    ${CMAKE_SOURCE_DIR}/Src/dac.c
    ${CMAKE_SOURCE_DIR}/Src/daq.c
    ${CMAKE_SOURCE_DIR}/Src/error.c
    ${CMAKE_SOURCE_DIR}/Src/gpio.c
    ${CMAKE_SOURCE_DIR}/Src/i2c.c
    ${CMAKE_SOURCE_DIR}/Src/lut.c
    ${CMAKE_SOURCE_DIR}/Src/main.c
    ${CMAKE_SOURCE_DIR}/Src/mc_power.c
    ${CMAKE_SOURCE_DIR}/Src/motors_helper.c
    ${CMAKE_SOURCE_DIR}/Src/motors.c
    ${CMAKE_SOURCE_DIR}/Src/movella.c
    ${CMAKE_SOURCE_DIR}/Src/pumps.c
    ${CMAKE_SOURCE_DIR}/Src/pwm.c
    ${CMAKE_SOURCE_DIR}/Src/safety_filter.c
    ${CMAKE_SOURCE_DIR}/Src/sensoric.c
    ${CMAKE_SOURCE_DIR}/Src/sensors.c
    ${CMAKE_SOURCE_DIR}/Src/state.c
    ${CMAKE_SOURCE_DIR}/Src/statusLED.c
    ${CMAKE_SOURCE_DIR}/Src/steering.c
    ${CMAKE_SOURCE_DIR}/Src/tssi.c
    ${CMAKE_SOURCE_DIR}/Src/syscalls.c
    ${CMAKE_SOURCE_DIR}/Src/sysmem.c


    # =========================
    # Optimizer source files
    # =========================

    ${CMAKE_SOURCE_DIR}/optimizer/optimizer.c
    ${CMAKE_SOURCE_DIR}/optimizer/qform.c
)


add_dependencies(${CMAKE_PROJECT_NAME} generate_gitcommit)


# Add additional sources
target_sources(${CMAKE_PROJECT_NAME}
    PUBLIC
    ${sources_SRCS}
)


# Include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE

    ${include_DIRS}

    $<$<COMPILE_LANGUAGE:C>:
        ${include_c_DIRS}
    >

    $<$<COMPILE_LANGUAGE:CXX>:
        ${include_cxx_DIRS}
    >

    $<$<COMPILE_LANGUAGE:ASM>:
        ${include_asm_DIRS}
    >
)


# Compile definitions
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE

    ${symbols_SYMB}

    $<$<COMPILE_LANGUAGE:C>:
        ${symbols_c_SYMB}
    >

    $<$<COMPILE_LANGUAGE:CXX>:
        ${symbols_cxx_SYMB}
    >

    $<$<COMPILE_LANGUAGE:ASM>:
        ${symbols_asm_SYMB}
    >

    # Configuration specific
    $<$<CONFIG:Debug>:DEBUG>
    $<$<CONFIG:Release>: >
)


# CPU compiler options
add_compile_options(${cpu_PARAMS})


# STM32F413 drivers
add_subdirectory(
    ${CMAKE_CURRENT_SOURCE_DIR}/../stm32f413-drivers
    ${CMAKE_BINARY_DIR}/stm32f413-drivers
)


# Link libraries
target_link_libraries(
    ${CMAKE_PROJECT_NAME}
    ${link_LIBS}
)


# Compiler options
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE

    ${cpu_PARAMS}
    ${compiler_OPTS}

    -Wall
    -Wshadow
    -Wextra

    -Wformat-overflow
    -Wformat-truncation

    -Wdouble-promotion

    -fno-common

    -Wundef

    $<$<COMPILE_LANGUAGE:C>:
        -ffunction-sections
        -fdata-sections
        -fstack-usage
    >

    $<$<COMPILE_LANGUAGE:CXX>:

    >

    $<$<COMPILE_LANGUAGE:ASM>:
        -x assembler-with-cpp
        -MMD
        -MP
    >

    $<$<CONFIG:Static>:
        -fanalyzer
        -fdiagnostics-format=sarif-file
    >

    $<$<CONFIG:Debug>:
        -Og
        -g3
        -ggdb
    >

    $<$<CONFIG:Release>:
        -Og
        -g0
    >
)


# Linker options
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE

    -T${linker_script_SRC}

    ${cpu_PARAMS}
    ${linker_OPTS}

    -Wl,-Map=${CMAKE_PROJECT_NAME}.map

    -Wl,--gc-sections

    -static

    --specs=nano.specs

    -Wl,--start-group
    -lc
    -lm
    -lstdc++
    -lsupc++
    -Wl,--end-group

    -Wl,-z,max-page-size=8

    -Wl,--print-memory-usage
)


# Post-build commands
add_custom_command(
    TARGET ${CMAKE_PROJECT_NAME}
    POST_BUILD

    COMMAND ${CMAKE_SIZE}
            $<TARGET_FILE:${CMAKE_PROJECT_NAME}>

    COMMAND ${CMAKE_OBJCOPY}
            -O ihex
            $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            ${CMAKE_PROJECT_NAME}.hex

    COMMAND ${CMAKE_OBJCOPY}
            -O binary
            $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            ${CMAKE_PROJECT_NAME}.bin
)