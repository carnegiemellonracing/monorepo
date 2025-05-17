cmake_minimum_required(VERSION 3.24)

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
project(VSM)
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
# Core MCU flags, CPU type, instruction set and FPU setup
set(cpu_PARAMS

    # Other parameters
    # -mthumb
    # -mcpu, -mfloat, -mfloat-abi, ...
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard

)


# Sources
set(sources_SRCS)
set(PROCESSOR F413)
set(PROCESSOR_FAM F4XX)

# Include directories for all compilers
set(include_DIRS
    ${CMAKE_SOURCE_DIR}/Inc
)

# Include directories for each compiler
set(include_c_DIRS)
set(include_cxx_DIRS)
set(include_asm_DIRS)

# Symbols definition for all compilers
set(symbols_SYMB
    STM32F413xx
    ${PROCESSOR}
)

add_library(freertos_config INTERFACE)
target_include_directories(freertos_config SYSTEM
    INTERFACE
    ${CMAKE_SOURCE_DIR}/Inc
    ${CMAKE_SOURCE_DIR}/../stm32f413-drivers/CMR/include) # The config file directory
target_compile_definitions(freertos_config
    INTERFACE
    configASSERT
    projCOVERAGE_TEST=0)

set(FREERTOS_PORT "GCC_ARM_CM4F" CACHE STRING "" FORCE)
# Symbols definition for each compiler
set(symbols_c_SYMB
    configASSERT
)
set(symbols_cxx_SYMB)
set(symbols_asm_SYMB)

# Link directories and names of libraries
set(link_DIRS)
set(link_LIBS
    stm-drivers
)

# Linker script
set(linker_script_SRC)

# Compiler options

set (compiler_OPTS)

set(compiler_OPTS
    -mfp16-format=ieee
    --specs=nano.specs
)

# Linker options
set(linker_OPTS)

# Now call generated cmake
# This will add script generated
# information to the project

include("cmake/vscode_generated.cmake")
# TODO We should really add Target groups or something

# Link directories setup
# Must be before executable is added
link_directories(${CMAKE_PROJECT_NAME} ${link_DIRS})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
    # Project sources
    ${CMAKE_SOURCE_DIR}/Src/adc.c
    ${CMAKE_SOURCE_DIR}/Src/can.c
    ${CMAKE_SOURCE_DIR}/Src/gpio.c
    ${CMAKE_SOURCE_DIR}/Src/main.c
    ${CMAKE_SOURCE_DIR}/Src/error.c
    ${CMAKE_SOURCE_DIR}/Src/state.c
    ${CMAKE_SOURCE_DIR}/Src/sensors.c
    ${CMAKE_SOURCE_DIR}/Src/syscall.c
    ${CMAKE_SOURCE_DIR}/Src/sysmem.c

)
add_dependencies(${CMAKE_PROJECT_NAME} generate_gitcommit)
# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PUBLIC ${sources_SRCS})

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${include_DIRS}
    $<$<COMPILE_LANGUAGE:C>: ${include_c_DIRS}>
    $<$<COMPILE_LANGUAGE:CXX>: ${include_cxx_DIRS}>
    $<$<COMPILE_LANGUAGE:ASM>: ${include_asm_DIRS}>
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
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../stm32f413-drivers ${CMAKE_BINARY_DIR}/stm32f413-drivers)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME} ${link_LIBS})

# Compiler options
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    ${cpu_PARAMS}
    ${compiler_OPTS}
    -Wall
    -Wshadow
    -Wextra
    -Wpedantic
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