cmake_minimum_required(VERSION 3.24)

project(HVCStateTests C)

set(CMAKE_C_STANDARD 23)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

enable_testing()

set(UNITY_ROOT
    "${CMAKE_CURRENT_SOURCE_DIR}/../../software_test_vendors/Unity"
    CACHE PATH "Path to a Unity checkout or vendored Unity directory"
)
set(FFF_ROOT
    "${CMAKE_CURRENT_SOURCE_DIR}/../../software_test_vendors/fff"
    CACHE PATH "Path to an FFF checkout or vendored FFF directory"
)

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

add_executable(test_hvc_state
    test_state_task_transitions.c
    "${UNITY_SOURCE}"
)

target_include_directories(test_hvc_state PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/fakes/include"
    "${UNITY_INCLUDE_DIR}"
    "${FFF_INCLUDE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/../Inc"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../stm32f413-drivers/CMR/include"
)

target_compile_definitions(test_hvc_state PRIVATE
    UNIT_TEST
)

target_compile_options(test_hvc_state PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)

add_test(NAME test_hvc_state COMMAND test_hvc_state)
