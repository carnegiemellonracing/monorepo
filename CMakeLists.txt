cmake_minimum_required(VERSION 3.24)

set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")
set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

project(25e)

# Needed for ExternalProject_Add
include(ExternalProject)

set(SCRIPTS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/stm32f413-drivers/filegen")

find_package(Python COMPONENTS Interpreter QUIET)

if(Python_FOUND)
    execute_process(
        COMMAND ${Python_EXECUTABLE} -c "import json, re"
        RESULT_VARIABLE PYTHON_DEPS_CHECK
        OUTPUT_QUIET
        ERROR_QUIET
    )
    if(PYTHON_DEPS_CHECK EQUAL 0)
        set(PYTHON_DEPS_AVAILABLE TRUE)
    else()
        set(PYTHON_DEPS_AVAILABLE FALSE)
    endif()
else()
    set(PYTHON_DEPS_AVAILABLE FALSE)
endif()

if(PYTHON_DEPS_AVAILABLE)
    add_custom_target(run_codegen_scripts ALL
        COMMAND ${Python_EXECUTABLE} "${SCRIPTS_DIR}/canid_mapper.py"
        COMMAND ${Python_EXECUTABLE} "${SCRIPTS_DIR}/extra_info.py"
        COMMAND ${Python_EXECUTABLE} "${SCRIPTS_DIR}/sym_gen.py"
        COMMAND ${Python_EXECUTABLE} "${SCRIPTS_DIR}/merge_can_signals.py"
        COMMAND ${Python_EXECUTABLE} "${SCRIPTS_DIR}/enum_generator.py"
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        COMMENT "Running code generation scripts"
        VERBATIM
    )
else()
    add_custom_target(run_codegen_scripts
        COMMAND ${CMAKE_COMMAND} -E echo "WARNING: Symbol files not generated - Python or dependencies missing."
        COMMENT "Skipping code generation scripts"
        VERBATIM
    )
endif()


ExternalProject_Add(
    HVC
    PREFIX ${CMAKE_CURRENT_BINARY_DIR}/HVC
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/HVC
    INSTALL_COMMAND ""
    BUILD_ALWAYS TRUE
    CMAKE_ARGS -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
)

ExternalProject_Add(
    RAM
    PREFIX ${CMAKE_CURRENT_BINARY_DIR}/RAM
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/RAM
    INSTALL_COMMAND ""
    BUILD_ALWAYS TRUE
    CMAKE_ARGS -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
)

ExternalProject_Add(
    VSM
    PREFIX ${CMAKE_CURRENT_BINARY_DIR}/VSM
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/VSM
    INSTALL_COMMAND ""
    BUILD_ALWAYS TRUE
    CMAKE_ARGS -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
)

ExternalProject_Add(
        DIM
        PREFIX ${CMAKE_CURRENT_BINARY_DIR}/DIM
        SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/DIM
        INSTALL_COMMAND ""
        BUILD_ALWAYS TRUE
        CMAKE_ARGS -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
)

ExternalProject_Add(
        DCM
        PREFIX ${CMAKE_CURRENT_BINARY_DIR}/DCM
        SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/DCM
        INSTALL_COMMAND ""
        BUILD_ALWAYS TRUE
        CMAKE_ARGS -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
)

ExternalProject_Add(
        HVI
        PREFIX ${CMAKE_CURRENT_BINARY_DIR}/HVI
        SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/HVI
        INSTALL_COMMAND ""
        BUILD_ALWAYS TRUE
        CMAKE_ARGS -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
)

ExternalProject_Add(
        LV-BMS
        PREFIX ${CMAKE_CURRENT_BINARY_DIR}/LV-BMS
        SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/LV-BMS
        INSTALL_COMMAND ""
        BUILD_ALWAYS TRUE
        CMAKE_ARGS -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
)

ExternalProject_Add(
        CIA
        PREFIX ${CMAKE_CURRENT_BINARY_DIR}/CIA
        SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/CIA
        INSTALL_COMMAND ""
        BUILD_ALWAYS TRUE
        CMAKE_ARGS -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
)