cmake_minimum_required(VERSION 3.24)

#
# Parent config for Generic/Arm
#
set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE   STATIC_LIBRARY)

#
# User is free to modify the file as much as necessary
#
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)

# list(APPEND CMAKE_MODULE_PATH "C:/Users/user1/Downloads/CmakeTestProject/cmake")
list(APPEND CMAKE_MODULE_PATH "{{sr:cmake_path}}")
message("Build CMAKE_MODULE_PATH: " ${CMAKE_MODULE_PATH})
#Why does vscode read this when its included here?
include("cmake/gcc-arm-none-eabi.cmake")
message("Build CMAKE_MODULE_PATH: " ${CMAKE_MODULE_PATH})

# Core project settings
project(boot)
enable_language(C ASM)
message("Build type: " ${CMAKE_BUILD_TYPE})

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_COLOR_DIAGNOSTICS ON)

if(CMAKE_HOST_WIN32)
add_custom_target(generate_gitcommit ALL
    COMMAND powershell -ExecutionPolicy Bypass -File ${CMAKE_CURRENT_SOURCE_DIR}/export_git.ps1
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

# Processor configuration
set(PROCESSOR F413)
set(PROCESSOR_FAM F4XX)

# Core MCU flags, CPU type, instruction set and FPU setup
set(cpu_PARAMS
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
)

# ---------------------------------------------------------------------------
# Paths — this CMakeLists.txt sits alongside the Makefile, so the OpenBLT
# Source tree is three levels up.
# ---------------------------------------------------------------------------
set(OPENBLT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/openblt/Target/Source")
set(UIP_ROOT     "${OPENBLT_ROOT}/third_party/uip")
set(TUSB_ROOT    "${OPENBLT_ROOT}/third_party/tinyusb/src")

# ---------------------------------------------------------------------------
# Sources
# ---------------------------------------------------------------------------

# Local application files (same directory as this CMakeLists.txt)
# flash_layout.c is #included directly by another .c file — exclude it.
file(GLOB_RECURSE local_SRCS
    "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.s"
)
list(FILTER local_SRCS EXCLUDE REGEX "flash_layout\\.c$")

# OpenBLT core
file(GLOB core_SRCS         "${OPENBLT_ROOT}/*.c")

# Port: ARMCM4_STM32F4
file(GLOB port_SRCS
    "${OPENBLT_ROOT}/ARMCM4_STM32F4/*.c"
    "${OPENBLT_ROOT}/ARMCM4_STM32F4/GCC/*.c"
)

set(sources_SRCS
    ${local_SRCS}
    ${core_SRCS}
    ${port_SRCS}
)

# ---------------------------------------------------------------------------
# Include directories
# ---------------------------------------------------------------------------
set(include_DIRS
    # Local
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc
    # OpenBLT core
    ${OPENBLT_ROOT}
    ${OPENBLT_ROOT}/ARMCM4_STM32F4
    ${OPENBLT_ROOT}/ARMCM4_STM32F4/GCC
)

set(include_c_DIRS)
set(include_asm_DIRS)

# ---------------------------------------------------------------------------
# Symbols / defines
# ---------------------------------------------------------------------------
set(symbols_SYMB
    USE_FULL_LL_DRIVER
    USE_HAL_DRIVER
    STM32F429xx
    "__weak=__attribute__((weak))"
    "__packed=__attribute__((__packed__))"
)

set(symbols_c_SYMB)
set(symbols_asm_SYMB)

# Link directories and names of libraries
set(link_DIRS)
set(link_LIBS
    stm-drivers
)

# FreeRTOS config target (required by stm32f413-drivers even if not used)
add_library(freertos_config INTERFACE)
target_include_directories(freertos_config SYSTEM
    INTERFACE
    ${CMAKE_SOURCE_DIR}/Inc)
target_compile_definitions(freertos_config
    INTERFACE
    projCOVERAGE_TEST=0)

# ---------------------------------------------------------------------------
# Linker script
# ---------------------------------------------------------------------------
set(linker_script_SRC "${CMAKE_CURRENT_SOURCE_DIR}/stm32f413rgtx_FLASH.ld")

# ---------------------------------------------------------------------------
# Compiler / linker options
# ---------------------------------------------------------------------------
set(compiler_OPTS
    --specs=nano.specs
)

set(linker_OPTS)

# ---------------------------------------------------------------------------
# Executable
# ---------------------------------------------------------------------------
add_executable(${CMAKE_PROJECT_NAME} ${sources_SRCS})

# Include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${include_DIRS}
    $<$<COMPILE_LANGUAGE:C>:   ${include_c_DIRS}>
    $<$<COMPILE_LANGUAGE:ASM>: ${include_asm_DIRS}>
)

# Preprocessor symbols
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    ${symbols_SYMB}
    $<$<COMPILE_LANGUAGE:C>:   ${symbols_c_SYMB}>
    $<$<COMPILE_LANGUAGE:ASM>: ${symbols_asm_SYMB}>
    $<$<CONFIG:Debug>:   DEBUG>
    $<$<CONFIG:Release>: >
)

add_compile_options(${cpu_PARAMS})
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../stm32f413-drivers ${CMAKE_BINARY_DIR}/stm32f413-drivers)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME} ${link_LIBS})

# Compiler options
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    ${cpu_PARAMS}
    ${compiler_OPTS}
    -Wall
    -Wextra
    -Wshadow
    -Wdouble-promotion          # Doubles do not natively use the FPU on Cortex-M4F
    -Wformat-overflow           # Detects buffer overflow in sprintf/snprintf
    -Wformat-truncation         # Detects buffer truncation in sprintf/snprintf
    -fno-strict-aliasing
    $<$<COMPILE_LANGUAGE:C>:
        -ffunction-sections     # Place each function in its own .text section
        -fdata-sections
        -fstack-usage
    >
    $<$<COMPILE_LANGUAGE:ASM>: -x assembler-with-cpp -MMD -MP>
    $<$<CONFIG:Debug>:   -Og -g3 -ggdb>
    $<$<CONFIG:Release>: -Og -g0>
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
    -Wl,--end-group
    -Wl,--print-memory-usage
)

# ---------------------------------------------------------------------------
# Post-build: size report + S-record (mirrors the Makefile)
# ---------------------------------------------------------------------------
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_SIZE}     $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
    COMMAND ${CMAKE_OBJCOPY} -O srec
            $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            ${CMAKE_PROJECT_NAME}.srec
    COMMAND ${CMAKE_OBJDUMP} -x
            $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            > ${CMAKE_PROJECT_NAME}.dump
)