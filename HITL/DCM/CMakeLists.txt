cmake_minimum_required(VERSION 3.24)

set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)

list(APPEND CMAKE_MODULE_PATH "{{sr:cmake_path}}")
message("Build CMAKE_MODULE_PATH: " ${CMAKE_MODULE_PATH})
include("${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/gcc-arm-none-eabi.cmake")
message("Build CMAKE_MODULE_PATH: " ${CMAKE_MODULE_PATH})

project(DCM_HITL)
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

set(PROCESSOR H725)
set(PROCESSOR_FAM H7XX)

set(include_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc
)

set(include_c_DIRS)
set(include_cxx_DIRS)
set(include_asm_DIRS)

set(symbols_SYMB
    STM32H725xx
    ${PROCESSOR}
    DISABLE_PRECOMPILE_HEADERS_HITL
)

add_library(freertos_config INTERFACE)
target_include_directories(freertos_config SYSTEM
    INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../stm32f413-drivers/CMR/include
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

get_filename_component(MONOREPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(linker_script_SRC "${MONOREPO_ROOT}/DCM/STM32H725ZGTX_FLASH.ld")

message(STATUS "Linker script path: ${linker_script_SRC}")
set (compiler_OPTS)


set(compiler_OPTS
    -mfp16-format=ieee
    --specs=nano.specs
)

set(linker_OPTS)


#include("${CMAKE_SOURCE_DIR}/../../cmake/vscode_generated.cmake")

link_directories(${CMAKE_PROJECT_NAME} ${link_DIRS})

add_executable(${CMAKE_PROJECT_NAME}
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/can.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/gpio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/i2c.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/pwm.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../DCM/Startup/startup_stm32h725xx.s
)


target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${include_DIRS}
    $<$<COMPILE_LANGUAGE:C>: ${include_c_DIRS}>
    $<$<COMPILE_LANGUAGE:CXX>: ${include_cxx_DIRS}>
    $<$<COMPILE_LANGUAGE:ASM>: ${include_asm_DIRS}>
    ${CMAKE_CURRENT_SOURCE_DIR}/../../stm32f413-drivers/CMR/include
    # ${CMAKE_CURRENT_SOURCE_DIR}/../../DCM/Inc
    # ${CMAKE_CURRENT_SOURCE_DIR}/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    ${symbols_SYMB}
    $<$<COMPILE_LANGUAGE:C>: ${symbols_c_SYMB}>
    $<$<COMPILE_LANGUAGE:CXX>: ${symbols_cxx_SYMB}>
    $<$<COMPILE_LANGUAGE:ASM>: ${symbols_asm_SYMB}>

    # Configuration specific
    $<$<CONFIG:Debug>:DEBUG>
    $<$<CONFIG:Release>: >
)

add_compile_options(${cpu_PARAMS})
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../stm32f413-drivers ${CMAKE_BINARY_DIR}/stm32f413-drivers)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME} ${link_LIBS})

# Compiler options
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    ${cpu_PARAMS}
    ${compiler_OPTS}
    -Wall
    -Wshadow
    -Wextra
    -Wformat-overflow # Detects buffer overflow (sprintf, snprintf, etc.)
    -Wformat-truncation # Detects buffer truncation (sprintf, snprintf, etc.)
    -Wdouble-promotion # Doubles do not natively use the FPU on a Cortex-M4F
    -fno-common # Do not allow common global symbols
    -Wundef # Detects undefined macros which would silently evaluate to 0
    $<$<COMPILE_LANGUAGE:C>:
    -ffunction-sections # Place each function in its own section in .text
    -fdata-sections
    -fstack-usage
    >
    $<$<COMPILE_LANGUAGE:CXX>:>
    $<$<COMPILE_LANGUAGE:ASM>:-x assembler-with-cpp -MMD -MP>
    $<$<CONFIG:Static>:
    -fanalyzer
    -fdiagnostics-format=sarif-file
    >
    $<$<CONFIG:Debug>:-Og -g3 -ggdb>
    $<$<CONFIG:Release>:-Og -g0>
)

# Linker options
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -T${linker_script_SRC}
    ${cpu_PARAMS}
    ${linker_OPTS}
    -Wl,-Map=${CMAKE_PROJECT_NAME}.map
    -Wl,--gc-sections #Optimize unused sections flagged by -ffunction-sections
    -static
    --specs=nano.specs
    -Wl,--start-group
    -lc
    -lm
    -lstdc++
    -lsupc++
    -Wl,--end-group
    -Wl,-z,max-page-size=8 # Allow good software remapping across address space (with proper GCC section making)
    -Wl,--print-memory-usage
)

# Execute post-build to print size, generate hex and bin
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
)