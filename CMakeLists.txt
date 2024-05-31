cmake_minimum_required(VERSION 3.22)

#
# User is free to modify the file as much as necessary
#

#list(APPEND CMAKE_MODULE_PATH "C:/Users/user1/Downloads/CmakeTestProject/cmake")
list(APPEND CMAKE_MODULE_PATH "{{sr:cmake_path}}")
message("Build CMAKE_MODULE_PATH: " ${CMAKE_MODULE_PATH})
include("cmake/gcc-arm-none-eabi.cmake")
message("Build CMAKE_MODULE_PATH: " ${CMAKE_MODULE_PATH})

# Core project settings
project(RAM)
enable_language(C CXX ASM)
message("Build type: " ${CMAKE_BUILD_TYPE})

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

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
# Include directories for all compilers
set(include_DIRS
${CMAKE_SOURCE_DIR}/Inc
${CMAKE_SOURCE_DIR}/Gen
${CMAKE_SOURCE_DIR}/stm32f413-drivers
${CMAKE_SOURCE_DIR}/stm32f413-drivers/CMSIS/Include
${CMAKE_SOURCE_DIR}/stm32f413-drivers/CMSIS/Device/${PROCESSOR}
${CMAKE_SOURCE_DIR}/stm32f413-drivers/HAL/${PROCESSOR}/Inc
${CMAKE_SOURCE_DIR}/stm32f413-drivers/FreeRTOS/include
${CMAKE_SOURCE_DIR}/stm32f413-drivers/FreeRTOS/portable/GCC/ARM_CM4F
${CMAKE_SOURCE_DIR}/stm32f413-drivers/DIM-ESE
${CMAKE_SOURCE_DIR}/cn-cbor/include
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

# Symbols definition for each compiler
set(symbols_c_SYMB
configASSERT
)
set(symbols_cxx_SYMB)
set(symbols_asm_SYMB)

# Link directories and names of libraries
set(link_DIRS)
set(link_LIBS)

# Linker script
set(linker_script_SRC)

# Compiler options
set(compiler_OPTS
-mfp16-format=ieee
)

# Linker options
set(linker_OPTS)

# Now call generated cmake
# This will add script generated
# information to the project
include("cmake/vscode_generated.cmake")

# Link directories setup
# Must be before executable is added
link_directories(${CMAKE_PROJECT_NAME} ${link_DIRS})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
${CMAKE_SOURCE_DIR}/Src/can.c
${CMAKE_SOURCE_DIR}/Src/config.c
${CMAKE_SOURCE_DIR}/Src/gpio.c
${CMAKE_SOURCE_DIR}/Src/main.c
${CMAKE_SOURCE_DIR}/Src/parser_test.c
${CMAKE_SOURCE_DIR}/Src/parser.c
${CMAKE_SOURCE_DIR}/Src/sample.c
${CMAKE_SOURCE_DIR}/Src/syscall.c
${CMAKE_SOURCE_DIR}/Src/sysmem.c
${CMAKE_SOURCE_DIR}/Src/uart.c
# STM sources
${CMAKE_SOURCE_DIR}/stm32f413-drivers/CMR/adc.c
${CMAKE_SOURCE_DIR}/stm32f413-drivers/CMR/can.c
${CMAKE_SOURCE_DIR}/stm32f413-drivers/CMR/config_screen_helper.c
${CMAKE_SOURCE_DIR}/stm32f413-drivers/CMR/config.c
${CMAKE_SOURCE_DIR}/stm32f413-drivers/CMR/dma.c
${CMAKE_SOURCE_DIR}/stm32f413-drivers/CMR/dsp.c
${CMAKE_SOURCE_DIR}/stm32f413-drivers/CMR/f413.c
${CMAKE_SOURCE_DIR}/stm32f413-drivers/CMR/fir_filter.c
${CMAKE_SOURCE_DIR}/stm32f413-drivers/CMR/gpio.c
${CMAKE_SOURCE_DIR}/stm32f413-drivers/CMR/i2c.c
${CMAKE_SOURCE_DIR}/stm32f413-drivers/CMR/interrupts.c
${CMAKE_SOURCE_DIR}/stm32f413-drivers/CMR/l431.c
${CMAKE_SOURCE_DIR}/stm32f413-drivers/CMR/panic.c
${CMAKE_SOURCE_DIR}/stm32f413-drivers/CMR/pwm.c
${CMAKE_SOURCE_DIR}/stm32f413-drivers/CMR/qspi.c
${CMAKE_SOURCE_DIR}/stm32f413-drivers/CMR/rcc.c
${CMAKE_SOURCE_DIR}/stm32f413-drivers/CMR/spi.c
${CMAKE_SOURCE_DIR}/stm32f413-drivers/CMR/tasks.c
${CMAKE_SOURCE_DIR}/stm32f413-drivers/CMR/uart.c
${CMAKE_SOURCE_DIR}/stm32f413-drivers/CMR/watchdog.c

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

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME} ${link_LIBS})

# Compiler options
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    ${cpu_PARAMS}
    ${compiler_OPTS}
    -Wall
    # -Wextra
    # -Wpedantic
    -Wno-unused-parameter
    -Wno-unused-variable
    $<$<COMPILE_LANGUAGE:C>: >
    $<$<COMPILE_LANGUAGE:CXX>:

    # -Wno-volatile
    # -Wold-style-cast
    # -Wuseless-cast
    # -Wsuggest-override
    >
    $<$<COMPILE_LANGUAGE:ASM>:-x assembler-with-cpp -MMD -MP>
    $<$<CONFIG:Debug>:-Og -g3 -ggdb>
    $<$<CONFIG:Release>:-Og -g0>
)

# Linker options
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -T${linker_script_SRC}
    ${cpu_PARAMS}
    ${linker_OPTS}
    -Wl,-Map=${CMAKE_PROJECT_NAME}.map
    -u _printf_float # STDIO float formatting support (remove if not used)
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
