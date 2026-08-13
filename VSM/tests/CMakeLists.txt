cmake_minimum_required(VERSION 3.24)

project(VSMStateTests C)

set(CMAKE_C_STANDARD 23)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

enable_testing()

set(UNITY_ROOT
    "${CMAKE_CURRENT_SOURCE_DIR}/vendor/Unity"
    CACHE PATH "Path to a Unity checkout or vendored Unity directory"
)

set(FFF_ROOT
    "${CMAKE_CURRENT_SOURCE_DIR}/vendor/fff"
    CACHE PATH "Path to an FFF checkout or vendored FFF directory"
)

if(NOT UNITY_ROOT)
    message(FATAL_ERROR "Set UNITY_ROOT to a Unity directory, for example -DUNITY_ROOT=/path/to/Unity")
endif()

if(NOT FFF_ROOT)
    message(FATAL_ERROR "Set FFF_ROOT to an FFF directory, for example -DFFF_ROOT=/path/to/fff")
endif()

find_path(UNITY_INCLUDE_DIR
    NAMES unity.h
    PATHS
        "${UNITY_ROOT}/src"
        "${UNITY_ROOT}"
    NO_DEFAULT_PATH
    REQUIRED
)

find_file(UNITY_SOURCE
    NAMES unity.c
    PATHS
        "${UNITY_ROOT}/src"
        "${UNITY_ROOT}"
    NO_DEFAULT_PATH
    REQUIRED
)

find_path(FFF_INCLUDE_DIR
    NAMES fff.h
    PATHS
        "${FFF_ROOT}"
        "${FFF_ROOT}/src"
    NO_DEFAULT_PATH
    REQUIRED
)

add_executable(test_vsm_state
    test_state_transitions.c
    "${UNITY_SOURCE}"
)

target_include_directories(test_vsm_state PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/fakes/include"
    "${UNITY_INCLUDE_DIR}"
    "${FFF_INCLUDE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/../Inc"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../stm32f413-drivers/CMR/include"
)

target_compile_definitions(test_vsm_state PRIVATE
    UNIT_TEST
)

target_compile_options(test_vsm_state PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)

add_test(NAME test_vsm_state COMMAND test_vsm_state)
