cmake_minimum_required(VERSION 3.24)

#
# User is free to modify the file as much as necessary
#

# list(APPEND CMAKE_MODULE_PATH "C:/Users/user1/Downloads/CmakeTestProject/cmake")
list(APPEND CMAKE_MODULE_PATH "{{sr:cmake_path}}")
message("Build CMAKE_MODULE_PATH: " ${CMAKE_MODULE_PATH})
#Why does vscode readd this when its included here?
include("cmake/gcc-arm-none-eabi.cmake")

message("Build CMAKE_MODULE_PATH: " ${CMAKE_MODULE_PATH})

# Core project settings
project(RAM)
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
    ${CMAKE_SOURCE_DIR}/Gen
    ${CMAKE_SOURCE_DIR}/stm32f413-drivers
    ${CMAKE_SOURCE_DIR}/stm32f413-drivers/CMR
    ${CMAKE_SOURCE_DIR}/stm32f413-drivers/CMSIS/Include
    ${CMAKE_SOURCE_DIR}/stm32f413-drivers/CMSIS/Device/${PROCESSOR}
    ${CMAKE_SOURCE_DIR}/stm32f413-drivers/HAL/${PROCESSOR_FAM}/Inc
    ${CMAKE_SOURCE_DIR}/stm32f413-drivers/DIM-ESE
    ${CMAKE_SOURCE_DIR}/cJSON
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

set(FREERTOS_CONFIG_FILE_DIRECTORY ${CMAKE_SOURCE_DIR}/Inc CACHE INTERNAL "")

set(FREERTOS_PORT GCC_ARM_CM4F CACHE INTERNAL "")
# Symbols definition for each compiler
set(symbols_c_SYMB
    configASSERT
)
set(symbols_cxx_SYMB)
set(symbols_asm_SYMB)

# Link directories and names of libraries
set(link_DIRS)
set(link_LIBS
    freertos_kernel
    cn-cbor
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
# We should really add Target groups or something

# Link directories setup
# Must be before executable is added
link_directories(${CMAKE_PROJECT_NAME} ${link_DIRS})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
    # Project sources
    ${CMAKE_SOURCE_DIR}/Src/can.c
    ${CMAKE_SOURCE_DIR}/Src/config.c
    ${CMAKE_SOURCE_DIR}/Src/gpio.c
    ${CMAKE_SOURCE_DIR}/Src/main.c
    ${CMAKE_SOURCE_DIR}/Src/parser.c
    ${CMAKE_SOURCE_DIR}/Src/sample.c
    ${CMAKE_SOURCE_DIR}/Src/syscall.c
    ${CMAKE_SOURCE_DIR}/Src/sysmem.c
    ${CMAKE_SOURCE_DIR}/Src/uart.c

    # CMR sources
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
    #HAL
    ${CMAKE_SOURCE_DIR}/stm32f413-drivers/HAL/${PROCESSOR_FAM}/Src/stm32f4xx_hal.c
    ${CMAKE_SOURCE_DIR}/stm32f413-drivers/HAL/${PROCESSOR_FAM}/Src/stm32f4xx_hal_rcc.c
    ${CMAKE_SOURCE_DIR}/stm32f413-drivers/HAL/${PROCESSOR_FAM}/Src/stm32f4xx_hal_can.c
    ${CMAKE_SOURCE_DIR}/stm32f413-drivers/HAL/${PROCESSOR_FAM}/Src/stm32f4xx_hal_cortex.c
    ${CMAKE_SOURCE_DIR}/stm32f413-drivers/HAL/${PROCESSOR_FAM}/Src/stm32f4xx_hal_dma.c
    ${CMAKE_SOURCE_DIR}/stm32f413-drivers/HAL/${PROCESSOR_FAM}/Src/stm32f4xx_hal_gpio.c
    ${CMAKE_SOURCE_DIR}/stm32f413-drivers/HAL/${PROCESSOR_FAM}/Src/stm32f4xx_hal_i2c.c
    ${CMAKE_SOURCE_DIR}/stm32f413-drivers/HAL/${PROCESSOR_FAM}/Src/stm32f4xx_hal_pwr.c
    ${CMAKE_SOURCE_DIR}/stm32f413-drivers/HAL/${PROCESSOR_FAM}/Src/stm32f4xx_hal_rcc_ex.c
    ${CMAKE_SOURCE_DIR}/stm32f413-drivers/HAL/${PROCESSOR_FAM}/Src/stm32f4xx_hal_spi.c
    ${CMAKE_SOURCE_DIR}/stm32f413-drivers/HAL/${PROCESSOR_FAM}/Src/stm32f4xx_hal_uart.c
    ${CMAKE_SOURCE_DIR}/stm32f413-drivers/HAL/${PROCESSOR_FAM}/Src/stm32f4xx_hal_qspi.c
    ${CMAKE_SOURCE_DIR}/stm32f413-drivers/HAL/${PROCESSOR_FAM}/Src/stm32f4xx_hal_tim.c
    ${CMAKE_SOURCE_DIR}/stm32f413-drivers/HAL/${PROCESSOR_FAM}/Src/stm32f4xx_hal_tim_ex.c
    ${CMAKE_SOURCE_DIR}/stm32f413-drivers/HAL/${PROCESSOR_FAM}/Src/stm32f4xx_hal_pcd.c
    ${CMAKE_SOURCE_DIR}/stm32f413-drivers/HAL/${PROCESSOR_FAM}/Src/stm32f4xx_hal_pcd_ex.c
    ${CMAKE_SOURCE_DIR}/stm32f413-drivers/HAL/${PROCESSOR_FAM}/Src/stm32f4xx_hal_sd.c
    ${CMAKE_SOURCE_DIR}/stm32f413-drivers/HAL/${PROCESSOR_FAM}/Src/stm32f4xx_hal_sdram.c
    ${CMAKE_SOURCE_DIR}/stm32f413-drivers/HAL/${PROCESSOR_FAM}/Src/stm32f4xx_hal_sram.c
    ${CMAKE_SOURCE_DIR}/stm32f413-drivers/HAL/${PROCESSOR_FAM}/Src/stm32f4xx_hal_ltdc.c
    ${CMAKE_SOURCE_DIR}/stm32f413-drivers/HAL/${PROCESSOR_FAM}/Src/stm32f4xx_hal_ltdc_ex.c
    ${CMAKE_SOURCE_DIR}/stm32f413-drivers/HAL/${PROCESSOR_FAM}/Src/stm32f4xx_hal_dsi.c
    ${CMAKE_SOURCE_DIR}/stm32f413-drivers/HAL/${PROCESSOR_FAM}/Src/stm32f4xx_hal_cryp.c
    ${CMAKE_SOURCE_DIR}/stm32f413-drivers/HAL/${PROCESSOR_FAM}/Src/stm32f4xx_hal_cryp_ex.c
    ${CMAKE_SOURCE_DIR}/stm32f413-drivers/HAL/${PROCESSOR_FAM}/Src/stm32f4xx_hal_hash.c
    ${CMAKE_SOURCE_DIR}/stm32f413-drivers/HAL/${PROCESSOR_FAM}/Src/stm32f4xx_hal_hash_ex.c
    ${CMAKE_SOURCE_DIR}/stm32f413-drivers/HAL/${PROCESSOR_FAM}/Src/stm32f4xx_hal_rng.c
    ${CMAKE_SOURCE_DIR}/stm32f413-drivers/HAL/${PROCESSOR_FAM}/Src/stm32f4xx_hal_cortex.c
    ${CMAKE_SOURCE_DIR}/stm32f413-drivers/HAL/${PROCESSOR_FAM}/Src/stm32f4xx_hal_flash.c
    ${CMAKE_SOURCE_DIR}/stm32f413-drivers/HAL/${PROCESSOR_FAM}/Src/stm32f4xx_hal_flash_ex.c
    #CMSIS
    ${CMAKE_SOURCE_DIR}/stm32f413-drivers/CMSIS/system_stm32f4xx.c
    #cJSON
    ${CMAKE_SOURCE_DIR}/cJSON/cJSON.c
)

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
# FreeRTOS

add_compile_options(${cpu_PARAMS})
add_subdirectory(${CMAKE_SOURCE_DIR}/stm32f413-drivers/FreeRTOS)

add_subdirectory(${CMAKE_SOURCE_DIR}/cn-cbor)
set(coveralls OFF CACHE BOOL "Generate coveralls data")

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
    -O0
    -ffunction-sections # Place each function in its own section in .text
    -fdata-sections
    -fstack-usage
    >
    $<$<COMPILE_LANGUAGE:CXX>:>
    $<$<COMPILE_LANGUAGE:ASM>:-x assembler-with-cpp -MMD -MP>
    $<$<CONFIG:Static>:
    -fanalyzer
    -fdump-analyzer
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