cmake_minimum_required(VERSION 3.24)

set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")
set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)
project(25e)

# Needed for ExternalProject_Add
include(ExternalProject)

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