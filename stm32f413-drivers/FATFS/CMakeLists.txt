cmake_minimum_required(VERSION 3.15)

# Only build FATFS when targeting STM32F413
# if(NOT (TARGET_F413))
#     return()
# endif()

project(FATFS C)

# Collect FATFS sources
file(GLOB FATFS_APP
    "${CMAKE_CURRENT_SOURCE_DIR}/App/*.c"
)

file(GLOB FATFS_MIDDLEWARE
    "${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/option/*.c"
)

file(GLOB FATFS_TARGET
    "${CMAKE_CURRENT_SOURCE_DIR}/Target/*.c"
)

add_library(FATFS STATIC
    ${FATFS_APP}
    ${FATFS_MIDDLEWARE}
    ${FATFS_TARGET}
)

# Include directories
target_include_directories(FATFS PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Target
    ${CMAKE_CURRENT_SOURCE_DIR}/App
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/option
)

# FATFS depends on HAL
target_link_libraries(FATFS PUBLIC HAL)
